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

// ADMM: min sum_k w_k ||D_k p||_1  s.t. lo <= p <= hi (per-sample box).  Returns p.
std::vector<double> l1ChannelBounds(const std::vector<double>& c, const std::vector<double>& blo,
                                    const std::vector<double>& bhi, const L1OptimalParams& pr,
                                    const BandChol& chol, int iterations) {
    const std::size_t n = c.size();
    if (n < 4) return c;
    const double w[3] = {pr.w1, pr.w2, pr.w3};
    std::vector<double> p = c, q = c, sdual(n, 0.0);
    std::array<std::vector<double>, 3> z, u;
    for (int k = 0; k < 3; ++k) {
        z[k] = applyD(kStencils[k], p);
        u[k].assign(z[k].size(), 0.0);
    }
    for (int it = 0; it < iterations; ++it) {
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
            const double qnew = std::min(bhi[i], std::max(blo[i], p_hat + sdual[i]));
            sdual[i] += p_hat - qnew;
            q[i] = qnew;
        }
    }
    return p;
}

// Offline wrapper: constant box +-B around the raw channel.
std::vector<double> l1Channel(const std::vector<double>& c, double B,
                              const L1OptimalParams& pr, const BandChol& chol) {
    std::vector<double> blo(c.size()), bhi(c.size());
    for (std::size_t i = 0; i < c.size(); ++i) { blo[i] = c[i] - B; bhi[i] = c[i] + B; }
    return l1ChannelBounds(c, blo, bhi, pr, chol, pr.iterations);
}

// Real-time receding-horizon L1 (§8o): repeatedly solve a small window
// [past P | commit K | future F] with already-committed samples pinned by a zero-width box
// (continuity anchor), commit K samples, slide. Streamable with an F-frame future buffer;
// each window is small so rt_iterations converge far faster than the global solve.
std::vector<double> l1ChannelRealtime(const std::vector<double>& ch, double B,
                                      const L1OptimalParams& pr, std::size_t P, std::size_t F,
                                      std::size_t K) {
    const std::size_t n = ch.size();
    if (n < 4) return ch;
    std::vector<double> out(n);
    std::size_t committed = 0;
    while (committed < n) {
        const std::size_t lo_i = committed > P ? committed - P : 0;
        const std::size_t hi_i = std::min(n, committed + K + F);   // exclusive
        const std::size_t W = hi_i - lo_i;
        if (W < 4) {  // tiny tail: pass raw through (cannot form the stencils)
            for (std::size_t g = committed; g < n; ++g) out[g] = ch[g];
            break;
        }
        std::vector<double> sub(W), blo(W), bhi(W);
        for (std::size_t j = 0; j < W; ++j) {
            const std::size_t g = lo_i + j;
            sub[j] = ch[g];
            if (g < committed) { blo[j] = out[g]; bhi[j] = out[g]; }  // pinned history
            else { blo[j] = ch[g] - B; bhi[j] = ch[g] + B; }
        }
        const BandChol chol(W);
        const std::vector<double> sol = l1ChannelBounds(sub, blo, bhi, pr, chol, pr.rt_iterations);
        const std::size_t ncommit = std::min(K, n - committed);
        for (std::size_t j = 0; j < ncommit; ++j) out[committed + j] = sol[committed - lo_i + j];
        committed += ncommit;
    }
    return out;
}

// Frame-cadence euler channels of a quat series (ts = f*1000/fps), unwrapped.
struct FrameChannels {
    std::vector<double> ts;
    std::array<std::vector<double>, 3> ch;
};

bool sampleFrameChannels(const std::vector<TimeQuat>& quats, double fps, FrameChannels& fc) {
    if (quats.size() < 4 || fps <= 0.0) return false;
    const double last_ts = quats.back().timestamp_ms;
    const std::size_t nf = static_cast<std::size_t>(std::floor(last_ts / 1000.0 * fps)) + 1;
    if (nf < 4) return false;
    fc.ts.resize(nf);
    for (auto& c : fc.ch) c.resize(nf);
    for (std::size_t f = 0; f < nf; ++f) {
        fc.ts[f] = static_cast<double>(f) * 1000.0 / fps;
        const Euler e = quatToEuler(sampleQuaternion(quats, fc.ts[f]));
        fc.ch[0][f] = e.roll;
        fc.ch[1][f] = e.pitch;
        fc.ch[2][f] = e.yaw;
    }
    for (auto& c : fc.ch) unwrap(c);
    return true;
}

std::vector<TimeQuat> recomposeChannels(const std::vector<double>& ts,
                                        const std::array<std::vector<double>, 3>& out) {
    std::vector<TimeQuat> result(ts.size());
    for (std::size_t f = 0; f < ts.size(); ++f) {
        result[f].timestamp_ms = ts[f];
        result[f].quat = eulerToQuat(out[0][f], out[1][f], out[2][f]).normalized();
    }
    return result;
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
    FrameChannels fc;
    if (!sampleFrameChannels(quats, fps, fc)) return quats;
    const std::size_t nf = fc.ts.size();

    std::array<std::vector<double>, 3> out;
    if (params.look_ahead_s >= 0.0) {
        // Real-time receding-horizon mode (§8o).
        const std::size_t P = static_cast<std::size_t>(std::max(4.0, params.past_s * fps));
        const std::size_t F = static_cast<std::size_t>(std::max(0.0, params.look_ahead_s * fps));
        const std::size_t K = static_cast<std::size_t>(std::max(1, params.commit_block));
        for (int k = 0; k < 3; ++k)
            out[k] = l1ChannelRealtime(fc.ch[k], params.max_deviation_deg[k] / kDeg, params, P, F, K);
    } else {
        const BandChol chol(nf);  // M depends only on nf — factor once, reuse per channel
        for (int k = 0; k < 3; ++k)
            out[k] = l1Channel(fc.ch[k], params.max_deviation_deg[k] / kDeg, params, chol);
    }
    return recomposeChannels(fc.ts, out);
}

std::vector<TimeQuat> smoothL1CropConstrained(const std::vector<TimeQuat>& quats, double fps,
                                              const L1OptimalParams& params, double max_zoom,
                                              const L1ReqZoomFn& reqzoom_fn,
                                              L1CropReport* report) {
    FrameChannels fc;
    if (!sampleFrameChannels(quats, fps, fc)) return quats;
    const std::size_t nf = fc.ts.size();
    const BandChol chol(nf);

    // Per-frame per-axis half-widths (rad), start at the full static budget.
    std::array<std::vector<double>, 3> b;
    for (int k = 0; k < 3; ++k) b[k].assign(nf, params.max_deviation_deg[k] / kDeg);

    // Tighten down to a small margin inside the clamp so demand shifted onto neighbouring
    // frames by the re-solve still lands under max_zoom.
    const double target = 1.0 + (max_zoom - 1.0) * 0.97;
    const double kMinBox = 0.02 / kDeg;  // never pin fully to raw (keeps the solver regular)
    const int kMaxOuter = 8;
    const std::size_t kHalo = 2;  // frames around a violation (interpolation + RS span)

    L1CropReport rep;
    std::array<std::vector<double>, 3> out;
    std::vector<TimeQuat> cand;
    std::vector<double> blo(nf), bhi(nf);
    for (int outer = 0;; ++outer) {
        for (int k = 0; k < 3; ++k) {
            for (std::size_t i = 0; i < nf; ++i) {
                blo[i] = fc.ch[k][i] - b[k][i];
                bhi[i] = fc.ch[k][i] + b[k][i];
            }
            out[k] = l1ChannelBounds(fc.ch[k], blo, bhi, params, chol, params.iterations);
        }
        cand = recomposeChannels(fc.ts, out);
        const std::vector<double> rz = reqzoom_fn(cand);
        const std::size_t nz = std::min(nf, rz.size());
        int breach = 0;
        double mx = 0.0;
        for (std::size_t f = 0; f < nz; ++f) {
            mx = std::max(mx, rz[f]);
            if (rz[f] > max_zoom) ++breach;
        }
        if (outer == 0) { rep.breach_before = breach; rep.max_reqz_before = mx; }
        rep.outer_iters = outer + 1;
        rep.breach_after = breach;
        rep.max_reqz_after = mx;
        if (outer >= kMaxOuter) break;
        // Shrink the boxes of frames above the (margined) target toward the deviation that
        // would meet it, scaling all three axes by the same factor (demand is ~linear in the
        // total deviation, §8p).
        bool any = false;
        for (std::size_t f = 0; f < nz; ++f) {
            if (rz[f] <= target) continue;
            any = true;
            const double s =
                std::min(0.95, std::max(0.25, (target - 1.0) / std::max(rz[f] - 1.0, 1e-9)));
            const std::size_t jlo = f >= kHalo ? f - kHalo : 0;
            const std::size_t jhi = std::min(nf - 1, f + kHalo);
            for (int k = 0; k < 3; ++k)
                for (std::size_t j = jlo; j <= jhi; ++j) {
                    const double dev = std::abs(out[k][j] - fc.ch[k][j]);
                    b[k][j] = std::max(kMinBox, std::min(b[k][j], dev * s));
                }
        }
        if (!any) break;
    }
    if (report) *report = rep;
    return cand;
}

} // namespace gyroflow
