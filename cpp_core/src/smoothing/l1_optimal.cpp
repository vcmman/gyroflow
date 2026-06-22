#include "gyroflow/smoothing/l1_optimal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace gyroflow {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = 180.0 / kPi;

// --- euler <-> quaternion, matching nalgebra (intrinsic X-Y-Z; same as default_algo) ---
struct Euler { double roll, pitch, yaw; };

Euler quatToEuler(const Quaternion& qin) {
    const Quaternion q = qin.normalized();
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    const double m20 = 2.0 * (x * z - w * y);
    if (std::abs(m20) < 1.0) {
        const double m21 = 2.0 * (y * z + w * x);
        const double m22 = 1.0 - 2.0 * (x * x + y * y);
        const double m10 = 2.0 * (x * y + w * z);
        const double m00 = 1.0 - 2.0 * (y * y + z * z);
        return {std::atan2(m21, m22), -std::asin(m20), std::atan2(m10, m00)};
    }
    const double m01 = 2.0 * (x * y - w * z);
    const double m02 = 2.0 * (x * z + w * y);
    if (m20 <= -1.0) return {std::atan2(m01, m02), kPi / 2.0, 0.0};
    return {std::atan2(-m01, -m02), -kPi / 2.0, 0.0};
}

Quaternion eulerToQuat(double roll, double pitch, double yaw) {
    const double sr = std::sin(roll * 0.5), cr = std::cos(roll * 0.5);
    const double sp = std::sin(pitch * 0.5), cp = std::cos(pitch * 0.5);
    const double sy = std::sin(yaw * 0.5), cy = std::cos(yaw * 0.5);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

// Unwrap a phase signal in place (remove ±2pi jumps), like numpy.unwrap.
void unwrap(std::vector<double>& a) {
    for (std::size_t i = 1; i < a.size(); ++i) {
        double d = a[i] - a[i - 1];
        while (d > kPi) { a[i] -= 2.0 * kPi; d = a[i] - a[i - 1]; }
        while (d < -kPi) { a[i] += 2.0 * kPi; d = a[i] - a[i - 1]; }
    }
}

// --- finite-difference operators (unit spacing) as explicit stencils ---
// D1 = [-1, 1], D2 = [1, -2, 1], D3 = [-1, 3, -3, 1].
const std::array<std::vector<double>, 3> kStencils = {
    std::vector<double>{-1.0, 1.0}, std::vector<double>{1.0, -2.0, 1.0},
    std::vector<double>{-1.0, 3.0, -3.0, 1.0}};

// y = D_k p   (length n - k)
std::vector<double> applyD(const std::vector<double>& s, const std::vector<double>& p) {
    const std::size_t L = s.size(), n = p.size();
    std::vector<double> out(n + 1 - L, 0.0);
    for (std::size_t r = 0; r < out.size(); ++r) {
        double v = 0.0;
        for (std::size_t a = 0; a < L; ++a) v += s[a] * p[r + a];
        out[r] = v;
    }
    return out;
}

// out = D_k^T y   (length n)
std::vector<double> applyDt(const std::vector<double>& s, const std::vector<double>& y,
                            std::size_t n) {
    const std::size_t L = s.size();
    std::vector<double> out(n, 0.0);
    for (std::size_t r = 0; r < y.size(); ++r)
        for (std::size_t a = 0; a < L; ++a) out[r + a] += s[a] * y[r];
    return out;
}

double softThreshold(double x, double t) {
    if (x > t) return x - t;
    if (x < -t) return x + t;
    return 0.0;
}

// Symmetric banded SPD matrix M = I + sum_k D_k^T D_k, half-bandwidth 3, stored lower:
// band[i][d] = M(i, i-d), d in [0,3]. Cholesky in place -> L with same layout.
struct BandChol {
    std::size_t n;
    std::vector<std::array<double, 4>> L;  // L[i][d] = L(i, i-d)

    explicit BandChol(std::size_t n_) : n(n_), L(n_, {0, 0, 0, 0}) {
        // assemble M
        for (std::size_t i = 0; i < n; ++i) L[i][0] = 1.0;  // identity
        for (const auto& s : kStencils) {
            const std::size_t Ln = s.size();
            if (n < Ln) continue;
            for (std::size_t r = 0; r + Ln <= n; ++r)
                for (std::size_t a = 0; a < Ln; ++a)
                    for (std::size_t b = 0; b < Ln; ++b) {
                        const std::size_t ii = r + a, jj = r + b;
                        if (ii >= jj && ii - jj <= 3) L[ii][ii - jj] += s[a] * s[b];
                    }
        }
        factor();
    }

    double at(std::size_t i, std::size_t j) const {  // current L value, lower triangle
        const std::size_t d = i - j;
        return d <= 3 ? L[i][d] : 0.0;
    }

    void factor() {
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t jlo = i >= 3 ? i - 3 : 0;
            for (std::size_t j = jlo; j <= i; ++j) {
                double s = L[i][i - j];  // M(i,j) (pre-factor) stored here initially
                const std::size_t klo = (i >= 3 ? i - 3 : 0);
                for (std::size_t k = klo; k < j; ++k) s -= at(i, k) * at(j, k);
                if (i == j) {
                    L[i][0] = std::sqrt(std::max(s, 1e-300));
                } else {
                    L[i][i - j] = s / L[j][0];
                }
            }
        }
    }

    // solve M x = b in place into x
    std::vector<double> solve(const std::vector<double>& b) const {
        std::vector<double> x(n);
        for (std::size_t i = 0; i < n; ++i) {  // forward: L y = b
            double s = b[i];
            for (std::size_t d = 1; d <= 3 && d <= i; ++d) s -= L[i][d] * x[i - d];
            x[i] = s / L[i][0];
        }
        for (std::size_t ii = n; ii-- > 0;) {  // back: L^T x = y
            double s = x[ii];
            for (std::size_t d = 1; d <= 3 && ii + d < n; ++d) s -= L[ii + d][d] * x[ii + d];
            x[ii] = s / L[ii][0];
        }
        return x;
    }
};

// ADMM: min sum_k w_k ||D_k p||_1  s.t. |p - c| <= B.  Returns p.
std::vector<double> l1Channel(const std::vector<double>& c, double B,
                              const L1OptimalParams& pr, const BandChol& chol) {
    const std::size_t n = c.size();
    if (n < 4) return c;
    const double w[3] = {pr.w1, pr.w2, pr.w3};
    std::vector<double> p = c, q = c, sdual(n, 0.0);
    std::array<std::vector<double>, 3> z, u;
    for (int k = 0; k < 3; ++k) {
        z[k] = applyD(kStencils[k], p);
        u[k].assign(z[k].size(), 0.0);
    }
    for (int it = 0; it < pr.iterations; ++it) {
        // p-update: M p = (q - s) + sum_k D_k^T (z_k - u_k)
        std::vector<double> rhs(n);
        for (std::size_t i = 0; i < n; ++i) rhs[i] = q[i] - sdual[i];
        for (int k = 0; k < 3; ++k) {
            std::vector<double> t(z[k].size());
            for (std::size_t i = 0; i < t.size(); ++i) t[i] = z[k][i] - u[k][i];
            const std::vector<double> dt = applyDt(kStencils[k], t, n);
            for (std::size_t i = 0; i < n; ++i) rhs[i] += dt[i];
        }
        p = chol.solve(rhs);
        const double a = pr.over_relax;
        // z/u updates (over-relaxed: Dp_hat = a*Dp + (1-a)*z_old)
        for (int k = 0; k < 3; ++k) {
            const std::vector<double> Dp = applyD(kStencils[k], p);
            const double thr = w[k] / pr.rho;
            for (std::size_t i = 0; i < Dp.size(); ++i) {
                const double Dp_hat = a * Dp[i] + (1.0 - a) * z[k][i];
                const double znew = softThreshold(Dp_hat + u[k][i], thr);
                u[k][i] += Dp_hat - znew;
                z[k][i] = znew;
            }
        }
        // q-update (box projection, over-relaxed) + dual
        for (std::size_t i = 0; i < n; ++i) {
            const double p_hat = a * p[i] + (1.0 - a) * q[i];
            const double lo = c[i] - B, hi = c[i] + B;
            const double qnew = std::min(hi, std::max(lo, p_hat + sdual[i]));
            sdual[i] += p_hat - qnew;
            q[i] = qnew;
        }
    }
    return p;
}

} // namespace

std::array<double, 3> frameEulerMaxDeviationDeg(const std::vector<TimeQuat>& a,
                                                const std::vector<TimeQuat>& b, double fps) {
    std::array<double, 3> dev{0.0, 0.0, 0.0};
    if (a.empty() || b.empty() || fps <= 0.0) return dev;
    const double last = std::min(a.back().timestamp_ms, b.back().timestamp_ms);
    const std::size_t nf = static_cast<std::size_t>(std::floor(last / 1000.0 * fps)) + 1;
    std::array<std::vector<double>, 3> ca, cb;
    for (int k = 0; k < 3; ++k) { ca[k].resize(nf); cb[k].resize(nf); }
    for (std::size_t f = 0; f < nf; ++f) {
        const double ts = static_cast<double>(f) * 1000.0 / fps;
        const Euler ea = quatToEuler(sampleQuaternion(a, ts));
        const Euler eb = quatToEuler(sampleQuaternion(b, ts));
        ca[0][f] = ea.roll; ca[1][f] = ea.pitch; ca[2][f] = ea.yaw;
        cb[0][f] = eb.roll; cb[1][f] = eb.pitch; cb[2][f] = eb.yaw;
    }
    for (int k = 0; k < 3; ++k) {
        unwrap(ca[k]);
        unwrap(cb[k]);
        for (std::size_t f = 0; f < nf; ++f)
            dev[k] = std::max(dev[k], std::abs(ca[k][f] - cb[k][f]) * kDeg);
    }
    return dev;
}

std::vector<TimeQuat> smoothL1Optimal(const std::vector<TimeQuat>& quats, double fps,
                                      const L1OptimalParams& params) {
    if (quats.size() < 4 || fps <= 0.0) return quats;
    const double last_ts = quats.back().timestamp_ms;
    const std::size_t nf = static_cast<std::size_t>(std::floor(last_ts / 1000.0 * fps)) + 1;
    if (nf < 4) return quats;

    // Sample the raw attitude at frame cadence and split into euler channels.
    std::vector<double> ts(nf);
    std::array<std::vector<double>, 3> ch;
    for (auto& c : ch) c.resize(nf);
    for (std::size_t f = 0; f < nf; ++f) {
        ts[f] = static_cast<double>(f) * 1000.0 / fps;
        const Euler e = quatToEuler(sampleQuaternion(quats, ts[f]));
        ch[0][f] = e.roll;
        ch[1][f] = e.pitch;
        ch[2][f] = e.yaw;
    }
    for (auto& c : ch) unwrap(c);

    const BandChol chol(nf);  // M depends only on nf — factor once, reuse per channel
    std::array<std::vector<double>, 3> out;
    for (int k = 0; k < 3; ++k)
        out[k] = l1Channel(ch[k], params.max_deviation_deg[k] / kDeg, params, chol);

    std::vector<TimeQuat> result(nf);
    for (std::size_t f = 0; f < nf; ++f) {
        result[f].timestamp_ms = ts[f];
        result[f].quat = eulerToQuat(out[0][f], out[1][f], out[2][f]).normalized();
    }
    return result;
}

} // namespace gyroflow
