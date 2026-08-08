# kilix-motion-detect

Frame-difference motion detection in C11: frames in, boxes out.

This exists because motion is the cheap gate in front of expensive work. Object
detection costs tens of milliseconds a frame; deciding whether anything moved
costs about two. Running the second only where the first found something is what
lets one machine watch several cameras.

Dependencies are a C11 compiler and libm. No image library, no threads, and no
allocation after setup.

## Build

```sh
make
make test
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

kmd_detector *detector;
kmd_detector_create(&detector, &config);

kmd_box boxes[32];
kmd_result result;
size_t count = kmd_detect(detector, frame, boxes, 32, &result);

if (!result.calibrating && count > 0) {
    /* something moved; boxes are in source coordinates */
}
```

Boxes come back in **source** coordinates, so a caller never has to know what
the module scaled to internally.

The scope is deliberately narrow: it reports where pixels changed and nothing
about what changed them. No classification, no tracking, and no grouping of
boxes into detector-sized regions — those depend on the consumer, so they belong
to the consumer.

## What it costs

Measured on an i7-6820HQ, 640×360 source, 300 frames:

| working height | ms/frame | vs full res | seven cameras @10fps |
| --- | --- | --- | --- |
| 360 (no downscale) | 19.8 | 1.0× | 139% of a core |
| 180 | 6.2 | 3.2× | 43% |
| **100** (default) | **2.3** | **8.8×** | **16%** |
| 64 | 1.4 | 14.4× | 10% |

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

The defaults match Frigate's, which have been run against far more cameras than
this. They are kept rather than tuned down because lowering the false-positive
rate necessarily costs recall, and measuring that trade needs footage with
verified movement in it — which this has not been tested against yet. Treat the
table as a tuning guide, not as a recommendation to change the defaults blind.

## Downscaling is not a detail

`area_average` is on by default and matters more than it sounds.

Point sampling takes one source pixel per output pixel. It is cheaper, and on a
detailed scene it is unusable: sub-pixel camera shake, or one noisy sensor pixel,
changes *which* source pixel is taken, and across a high-contrast edge the two
candidates differ by more than any sensible threshold. Measured on a still
workshop full of tools and cables, point sampling reported motion on **100% of
frames**; area averaging on the same clip reported 24%.

It costs about 30% more per frame. A gate that never closes saves nothing
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

A bitmap rather than polygons, on purpose: rasterising a polygon is a geometry
problem, this module is arithmetic on pixels, and a caller that has polygons
already has something that can draw them. It also keeps the module
dependency-free.

## License

MIT. See `LICENSE`.
