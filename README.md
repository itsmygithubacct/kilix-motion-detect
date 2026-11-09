# kilix-motion-detect

Frame-difference motion detection in C11: frames in, boxes out.

**Status:** a library, not an application. It is built into
[kilix-nvr](https://github.com/itsmygithubacct/kilix-nvr#readme) and
[kilix-object-detect](https://github.com/itsmygithubacct/kilix-object-detect#readme)
as the git submodule `third_party/kilix-motion-detect`, so it reaches a
Plebian-OS user when one of those apps is installed on demand from the Kilix
content catalog (`kilix install kilix-nvr`, `kilix install kilix-object-detect`).
It has no model and downloads nothing.

This exists because motion is the cheap gate in front of expensive work. Object
detection costs tens of milliseconds a frame; deciding whether anything moved
costs under one at the default working size on the benchmark host. Running the
second only where the first found something is what lets one machine watch
several cameras.

The only build dependency is a C11 toolchain. There is no image library and no
internal worker thread. `kmd_detect()` does not allocate; detector creation and
installing the first mask may allocate.

## What it does

- Takes grayscale, RGBA or BGRA frames of a fixed size and returns boxes, in
  source coordinates, around pixels that differ from a rolling background.
- Downscales to a small working height (100 by default) with area averaging,
  so cost per frame stays under a millisecond.
- Normalises contrast against recent frames, flags whole-scene lighting
  changes, and optionally skips such frames.
- Honours a per-pixel mask, so a noisy region can be ignored without raising
  the threshold everywhere.

## Build

```sh
make
make test
make fuzz      # deterministic randomized API and invariant checks
make sanitize
make bench      # cost per frame at several working sizes
```

`make stream` builds `kmd-stream`, which runs the detector over raw frames on
standard input and prints one line per frame with motion:

```sh
ffmpeg -i clip.mkv -f rawvideo -pix_fmt gray -vf scale=640:360 - \
  | kmd-stream 640 360 [detect_height] [contrast:0|1]
```

`make install` installs the static and shared libraries and the header.

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

## How it works

```
frame (gray / RGBA / BGRA, width x height)
  -> luma, downscaled to detect_height (area average)
  -> contrast stretch against a rolling window      (improve_contrast)
  -> blur
  -> difference against the background model, threshold
  -> scene-change / skip decision                   (lightning_threshold, skip_threshold)
  -> dilate -> connected components (>= contour_area)
  -> boxes projected back to exact source coordinates
```

The background model absorbs motion only after it persists, and a reset or a
mask change restarts calibration; boxes reported while `result.calibrating` is
true should be ignored. Every step works on preallocated buffers, so
`kmd_detect()` never allocates. Each stage's trade-offs are in
[docs/behaviour.md](docs/behaviour.md), and the measured cost, false-positive
and recall figures are in [docs/measurements.md](docs/measurements.md).

## Documentation

- [docs/measurements.md](docs/measurements.md): cost per frame at each working
  size, false positives on real footage, the threshold sweep and recall.
- [docs/behaviour.md](docs/behaviour.md): downscaling, contrast normalisation,
  lighting changes, masks, and the failure and lifecycle rules.
- [CHANGELOG.md](CHANGELOG.md): release history.

## Related repositories

- [kilix-nvr](https://github.com/itsmygithubacct/kilix-nvr#readme): the camera
  recorder that gates detection on motion.
- [kilix-object-detect](https://github.com/itsmygithubacct/kilix-object-detect#readme):
  `kilix-look`, which detects objects on crops around motion.
- [kilix-mask](https://github.com/itsmygithubacct/kilix-mask#readme): paints the
  region maps that become motion masks.
- [kilix-yolox](https://github.com/itsmygithubacct/kilix-yolox#readme): the
  object detector behind the gate.

## Part of Plebian-OS

kilix-motion-detect is one of the repositories behind
[Plebian-OS](https://github.com/itsmygithubacct/plebian-os#readme), a Debian
install whose desktop is [Kilix](https://github.com/itsmygithubacct/kilix#readme)
running the [Kilix 95](https://github.com/itsmygithubacct/kilix-95#readme)
desktop through [Pleb](https://github.com/itsmygithubacct/pleb#readme).
It ships inside the camera apps that use it, as a submodule. The map of every
repository and how it ships is on the
[components page](https://github.com/itsmygithubacct/plebian-os/blob/main/docs/components.md).

## License

MIT. See `LICENSE`.
