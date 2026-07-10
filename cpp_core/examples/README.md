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
  (EnvelopeFollower default, or GaussianFilter) → STAGE 3 `max_zoom` clamp → STAGE 4 (optional)
  crop-budget guard.

### STAGE 4 — crop-budget guard (zero black borders inside `max_zoom`)

STAGE 3 is the only step that can leave a black border: when a frame *demands* more zoom than
the clamp allows, capping the zoom leaves the overflow uncovered. The demand is created by the
**smoothing** (the smoothed path swinging too far from the real camera), so that is where the
guard fixes it — per frame, only where the budget overflows:

1. measure per-frame demand (STAGE 1) for the smoothed path and for a **reference** path that is
   always affordable (a barely-smoothed raw path, zero-phase EMA τ≈0.03 s);
2. demand envelope (centered window-max + zero-phase EMA + peak-hold) → gain
   `g[i] = (target − d_ref) / (env(d) − d_ref)` clamped to `[0,1]`, target a few % inside
   `max_zoom`. Envelope-speed gain = compressor (waveform-preserving); per-sample gain would be
   a clipper (harmonic distortion);
3. the smoothing side blends each frame toward the reference: `slerp(ref, smoothed, g)`
   (`ApplyGainFn` callback — the second codebase-specific piece);
4. one verify round re-measures and tightens locally.

The demo bursts the toy shake to 5× amplitude for 30 frames (real black wedges at 130 %):
`guard: rounds 1, breach 30 -> 0, max demand 1.736x -> 1.248x`.

Gyroflow the app ships a **native equivalent** (`max_zoom_iterations`, on by default): breaching
frames get their per-frame smoothing velocity limit scaled by {0.95…0.8} and the whole smoothing
re-runs, up to 5 rounds — same concept, actuated through the filter's adaptive α instead of a
post-blend. The guard form used here converges in one round and keeps the waveform clean; both
are analysed in `../SMOOTHING_RND.md` §8r (and the library implementation is
`../src/smoothing/crop_guard.cpp`, CLI `--fit-crop`).

Faithful to `../src/zooming/adaptive_zoom.cpp` (STAGES 0–3) and `../src/smoothing/crop_guard.cpp`
(STAGE 4); the algorithm's behaviour (static-vs-dynamic, per-config black border, both smoothing
methods, the guard's rendered verdict) is analysed in `../SMOOTHING_RND.md` §8b/§8c/§8c′/§8r
and `../PIPELINE.md`.
