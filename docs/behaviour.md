# Detector behaviour

Downscaling, contrast normalisation, lighting changes, masks, and the failure and lifecycle rules. Moved from the README.

## Downscaling is not a detail

`area_average` is on by default and matters more than it sounds.

Point sampling takes one source pixel per output pixel. It is cheaper, and on a
detailed scene it is unusable: sub-pixel camera shake, or one noisy sensor pixel,
changes *which* source pixel is taken, and across a high-contrast edge the two
candidates differ by more than any sensible threshold. Measured on a still
workshop full of tools and cables, point sampling reported motion on **100% of
frames**; area averaging on the same clip reported 24%.

At the default working size it costs about 60% more detector time than point
sampling (0.778 versus 0.480 ms/frame). A gate that never closes saves nothing
downstream, so it is on by default.

## Contrast normalisation

`improve_contrast` normalises against a rolling window of recent frames rather
than the current one, so a lighting change does not read as whole-frame motion.
It costs under 10% of frame time.

One consequence worth knowing: it stretches whatever range a frame has to fill
0–255, so a **flat** scene — a blank wall, heavy fog, a camera spanning only a
few grey levels — has its sensor noise amplified proportionally. A flat scene is
the hard case for this stage, not the easy one.

## Lighting changes

Two thresholds, and they do different things:

- `lightning_threshold` (default 0.8) — the frame is flagged `scene_changed` and
  the background model re-converges. **Boxes are still reported.**
- `skip_threshold` (default off) — boxes are suppressed entirely and the frame
  reports `skipped`.

Skipping is off by default deliberately. It is the only behaviour here that can
hide something real: a camera panning across a person reports no motion at all,
and a recorder gated on motion therefore stores nothing. That is a fair trade to
make knowingly and a poor one to inherit.

## Masks

`kmd_detector_set_mask()` takes one byte per source pixel — zero ignores it.

With area downscaling, a working cell remains active when any contributing
source pixel is active. Mask boundaries therefore have working-cell granularity:
mask the full source footprint of a cell to suppress it. The mask is copied, and
changing or clearing it resets background calibration.

A bitmap rather than polygons, on purpose: rasterising a polygon is a geometry
problem, this module is arithmetic on pixels, and a caller that has polygons
already has something that can draw them. It also keeps the module
dependency-free.

## Failure and lifecycle rules

`kmd_detector_create()` rejects invalid, non-finite, unrepresentable, or unsafe
configurations and leaves its output pointer null. `frame_alpha` must be finite
and in `(0, 1]`; the scene-change and skip fractions must be finite and in
`[0, 1]`. A blur radius larger than the working image is safe and is clamped to
the image extent.

Creation allocates all per-frame storage, so a successful `kmd_detect()` loop
cannot later fail for lack of memory. Setting the first mask is the one optional
post-creation allocation. `kmd_detector_reset()` and mask changes restart
calibration; consumers should ignore boxes while `result.calibrating` is true.

`make test` covers library behavior and stream-input error paths. `make fuzz`
adds deterministic randomized configurations, pixel formats, masks, resets,
capacities, and extreme blur radii. `make sanitize` runs both under
AddressSanitizer and UndefinedBehaviorSanitizer. `make fuzz` and `make bench`
fold every motion decision into a checksum that is held to committed baselines
in `tests/`, so a change that silently alters detection behavior fails the
suite; a deliberate behavior change re-pins them with `KMD_REBASELINE=1` in the
commit that explains it.
