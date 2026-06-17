// Verifies the Gyroflow "IMU orientation" setting for a gyro mounted in the OpenCV image
// camera frame (X=right/width, Y=down/height, Z=forward/optical-axis).
//
// CONCLUSION UNDER TEST: that setting must be "yxz" (NOT "Xyz" or "XYZ").
//
// The result is the composition of THREE coordinate remaps in the Rust pipeline:
//   1. imu_orientation `orient()`            src/core/gyro_source/imu_transforms.rs:73
//        out[i] = +/- in[axis]; 'X'->in0 'x'->-in0 'Y'->in1 'y'->-in1 'Z'->in2 'z'->-in2
//   2. a hardcoded swap inside EVERY integrator (complementary / VQF / simple / v2):
//        omega = (-g[1], g[0], g[2])         src/core/imu_integration/mod.rs:82,127,163,198,246,290
//   3. the render axis sign-flips = diag(1,-1,-1) applied to R(quat):
//        r[0][1]*=-1; r[0][2]*=-1; r[1][0]*=-1; r[2][0]*=-1
//        src/core/stabilization/frame_transform.rs:257  (== cpp_core frame_transform.cpp)
//
// Derivation: for stabilization, the angular velocity fed to the integrator (after orient +
// swap) must equal the physical gyro vector expressed in the orientation frame, which by (3)
// is S*omega with S=diag(1,-1,-1). So the required identity is
//        swap(orient(io, w_image)) == diag(1,-1,-1) * w_image      for all w_image,
// and equivalently, the render-flipped rotation matrix of the integrated quaternion must
// equal the physical camera rotation about the image-frame axis. This test checks both,
// using cpp_core's real quaternion->matrix code and the real flip pattern.

#include "gyroflow/mat3.hpp"
#include "gyroflow/types.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <string>

using namespace gyroflow;

namespace {

using V3 = std::array<double, 3>;

// imu_transforms.rs::orient
V3 orient(const std::string& io, const V3& in) {
    auto map = [&](char c) -> double {
        switch (c) {
            case 'X': return in[0];  case 'x': return -in[0];
            case 'Y': return in[1];  case 'y': return -in[1];
            case 'Z': return in[2];  case 'z': return -in[2];
        }
        assert(false && "bad orientation char");
        return 0.0;
    };
    return {map(io[0]), map(io[1]), map(io[2])};
}

// imu_integration/mod.rs: omega = (-g[1], g[0], g[2])
V3 integratorSwap(const V3& g) { return {-g[1], g[0], g[2]}; }

// frame_transform.rs:257 non-inverted-framebuffer flips (== cpp_core frame_transform.cpp).
Mat3 applyRenderFlips(Mat3 r) {
    r.at(0, 1) *= -1.0;
    r.at(0, 2) *= -1.0;
    r.at(1, 0) *= -1.0;
    r.at(2, 0) *= -1.0;
    return r;
}

bool matEq(const Mat3& a, const Mat3& b, double tol = 1e-9) {
    for (int i = 0; i < 9; ++i)
        if (std::abs(a.m[i] - b.m[i]) > tol) return false;
    return true;
}

V3 normalize(const V3& v) {
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return {v[0] / n, v[1] / n, v[2] / n};
}

// Does `io` correctly stabilize a gyro in the OpenCV image frame? For each image-frame axis
// (pitch/yaw/roll), the render-flipped rotation of the integrated quaternion must equal the
// physical camera rotation about that same image-frame axis.
bool stabilizes(const std::string& io) {
    const std::array<V3, 3> imageAxes = {V3{1, 0, 0}, V3{0, 1, 0}, V3{0, 0, 1}};
    const double theta = 0.21;  // arbitrary non-trivial angle
    for (const V3& wImage : imageAxes) {
        const V3 omega = integratorSwap(orient(io, wImage));
        const V3 axis = normalize(omega);
        const Mat3 R = Mat3::fromQuaternion(Quaternion::fromAxisAngle({axis[0], axis[1], axis[2]}, theta));
        const Mat3 rFlipped = applyRenderFlips(R);
        const Mat3 rPhysical =
            Mat3::fromQuaternion(Quaternion::fromAxisAngle({wImage[0], wImage[1], wImage[2]}, theta));
        if (!matEq(rFlipped, rPhysical)) return false;
    }
    return true;
}

} // namespace

int main() {
    // 1. Algebraic identity: swap(orient(io, w)) == diag(1,-1,-1) * w  iff io == "yxz".
    const V3 w = {0.3, -0.7, 1.1};  // arbitrary angular velocity in the image frame
    const V3 expected = {w[0], -w[1], -w[2]};  // S * w, S = diag(1,-1,-1)
    const V3 got = integratorSwap(orient("yxz", w));
    for (int i = 0; i < 3; ++i) assert(std::abs(got[i] - expected[i]) < 1e-12);

    // The two strings I previously (wrongly) suggested must NOT satisfy it.
    for (const char* bad : {"XYZ", "Xyz"}) {
        const V3 r = integratorSwap(orient(bad, w));
        bool same = true;
        for (int i = 0; i < 3; ++i) same &= std::abs(r[i] - expected[i]) < 1e-12;
        assert(!same && "only yxz should map an image-frame gyro into the orientation frame");
    }

    // 2. End-to-end via cpp_core's real quaternion/matrix + the real render flips:
    //    "yxz" stabilizes all three image-frame rotation axes; the others do not.
    assert(stabilizes("yxz"));
    assert(!stabilizes("XYZ"));
    assert(!stabilizes("Xyz"));
    // A couple more wrong candidates, for good measure.
    assert(!stabilizes("YXZ"));
    assert(!stabilizes("yxZ"));

    return 0;
}
