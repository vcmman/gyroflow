# Gyroflow C++ Core Port

This directory is the UI-free C++ porting surface for Gyroflow core algorithms.

Initial scope:

- Quaternion time series utilities.
- Quaternion-only stabilization path, matching the DJI-style fused attitude input first.
- Rolling shutter row timestamp and row compensation calculation.
- Lens profile data structures with `opencv_fisheye` as the default distortion model.

Near-term migration order:

1. DJI MP4 metadata reader: `djmd`/DVTM protobuf parsing into quaternion, lens profile, and readout metadata.
2. Lens distortion models: start with OpenCV fisheye, then add OpenCV standard and camera-specific models.
3. Default smoothing parity against Rust Gyroflow golden data.
4. CPU remap renderer for debug output.
5. Raw IMU integration paths: Complementary, VQF, SimpleGyro, Mahony, and Madgwick.

This is intentionally separate from QML/UI code. The target shape is a small core library plus CLI tools that can be validated against Rust Gyroflow outputs.
