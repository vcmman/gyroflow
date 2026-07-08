# examples/ — portable algorithm references

Standalone, dependency-free reference implementations meant for **porting to other codebases**
(e.g. board / embedded). Each file compiles and runs on its own with a built-in demo.

## `dynamic_zoom_reference.cpp` — dynamic ("adaptive") zoom

The full dynamic-zoom pipeline that decides, per frame, how much to crop/zoom a stabilized video
so black borders are removed with the least crop, varying smoothly over time.

```sh
g++ -std=c++11 -O2 dynamic_zoom_reference.cpp -o dz && ./dz
```

- **Input:** frame count, image/output dimensions, fps, smoothing window, `max_zoom`, method, and a
  `MapBorderFn` callback = your stabilization warp (lens undistort + smoothed rotation + rolling
  shutter). The callback is the **only** codebase-specific piece; the demo plugs in a toy rotation.
- **Output:** one FOV multiplier `fov[i] ∈ (0,1]` per frame — crop a centered `fov` fraction and
  upscale (zoom = `1/fov`).
- **Flow (marked in the source):** STAGE 0 border ring → STAGE 1 per-frame required FOV (largest
  border-free centered rectangle of the output aspect) → STAGE 2 temporal smoothing
  (EnvelopeFollower default, or GaussianFilter) → STAGE 3 `max_zoom` clamp.

Faithful to `../src/zooming/adaptive_zoom.cpp`; the algorithm's behaviour (static-vs-dynamic,
per-config black border, both smoothing methods) is analysed in `../SMOOTHING_RND.md` §8b/§8c/§8c′
and `../PIPELINE.md`.
