# kilix-motion-detect

Frame-difference motion detection in C11: frames in, boxes out.

This exists because motion is the cheap gate in front of expensive work. Object
detection costs tens of milliseconds a frame; deciding whether anything moved
costs under one at the default working size on the benchmark host. Running the
second only where the first found something is what lets one machine watch
several cameras.

The only build dependency is a C11 toolchain. There is no image library and no
internal worker thread. `kmd_detect()` does not allocate; detector creation and
installing the first mask may allocate.

## Build

```sh
make
make test
make fuzz      # deterministic randomized API and invariant checks
make sanitize
make bench      # cost per frame at several working sizes
```

## Use

```c
kmd_config config;
kmd_config_init(&config);
config.width = 640;
config.height = 360;
config.pixfmt = KMD_PIXFMT_BGRA;

kmd_detector *detector = NULL;
if (!kmd_detector_create(&detector, &config)) {
    /* invalid configuration or allocation failure */
    return false;
}

kmd_box boxes[32];
kmd_result result;
size_t count = kmd_detect(detector, frame, boxes, 32, &result);

if (!result.calibrating && count > 0) {
    /* something moved; boxes are in source coordinates */
}

kmd_detector_free(detector);
```

Each input is exactly `width * height` tightly packed pixels in the configured
format. The caller owns that bound because the frame pointer carries no size.
Boxes come back in exact, half-open **source** coordinates, so a caller never
has to know what the module scaled to internally.

Frames for one detector must arrive in order. Calls touching the same detector
need external synchronization; distinct detectors can run concurrently.

The scope is deliberately narrow: it reports where pixels changed and nothing
about what changed them. No classification, no tracking, and no grouping of
boxes into detector-sized regions — those depend on the consumer, so they belong
to the consumer.

## What it costs

Measured on an i7-6820HQ with a 640×360 source. These are the medians of seven
alternating baseline/candidate pairs over 300 detector calls per configuration;
scene generation is outside the timed region and every run produced the same
workload and motion-decision checksum.

| working height | ms/frame | vs full res | seven cameras @10fps |
| --- | --- | --- | --- |
| 360 (no downscale) | 8.593 | 1.0× | 60% of a core |
| 180 | 2.259 | 3.8× | 16% |
| **100** (default) | **0.778** | **11.0×** | **5.4%** |
| 64 | 0.365 | 23.5× | 2.6% |

`detect_height` is the largest lever by far, and it costs almost nothing in
sensitivity: dropping from full resolution to 100 pixels moves the smallest
detectable object from about four source pixels to about six.

## What it gets wrong, and how to tell

Measured against 22 days of real camera footage, on clips verified still by eye,
so every reported box is a false positive:

| camera | defaults | `threshold=55, contour_area=40` |
| --- | --- | --- |
| indoor workshop | 42%, 24% | 0%, 0.9% |
| driveway | 62%, 4% | 11%, 1% |
| back driveway | 0%, 0% | 0%, 0% |
| side camera | 0%, 0% | 0%, 0% |
| patio (IR, foliage) | 47%, 72% | 0%, 0% |
| pool (IR) | 14%, 30% | 0%, 0% |

**The false-positive rate is a property of the scene, not of the settings.** Two
of these six cameras are silent at the defaults; the rest are not, and a
cluttered indoor scene is the worst case by a wide margin. If motion is gating
something expensive, measure the camera before trusting the gate — a stage that
passes 70% of frames is not saving what you think it is.

The two dials, swept on the hardest scene:

| threshold ↓ / contour_area → | 10 | 20 | 40 | 80 |
| --- | --- | --- | --- | --- |
| 20 | 76% | 45% | 29% | 23% |
| **30** (default) | 24% | 21% | 12% | 0% |
| 40 | 12% | 11% | 6% | 5% |
| 55 | 3.5% | 2.6% | 0.9% | 0% |
| 70 | 2.6% | 1.7% | 0% | 0% |

### Recall, and why the defaults stay

Every row above lowers the false-positive rate, and every row must also lower
recall. Measured against 50 night events taken from a running recorder's own
event log — an index to clips where something actually happened, which random
sampling cannot supply:

| setting | recall | false positives |
| --- | --- | --- |
| **thr 30 / area 10** (default) | **78%** | 0–72%, scene-dependent |
| thr 40 / area 20 | 64% | ~11–21% |
| thr 55 / area 40 | **48%** | 0–1% |
| thr 70 / area 40 | 48% | 0% |

**No setting is both quiet and sensitive.** The configuration that looks best on
the false-positive table alone loses three in ten real events.

So the defaults stay where Frigate put them, and the answer to a noisy camera is
**a mask, not a higher threshold** — a mask removes the specific thing that
moves without costing sensitivity anywhere else in the frame. Treat the sweep as
a tuning guide, not a recommendation.

That recall figure is agreement with a mature detector rather than with truth,
so it inherits that detector's mistakes. Spot-checking misses found no visible
subject in several, which suggests 78% is a floor.

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

## License

MIT. See `LICENSE`.
