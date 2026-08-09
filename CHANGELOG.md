# Changelog

## 0.2.0

- Cut median detector time by 33–63% across the maintained benchmark matrix,
  including 40% at the default configuration, while preserving ordinary-frame
  motion decisions.
- Replaced radius-proportional blur and nine-probe dilation with bounded linear
  passes; large blur radii are now safe and remain proportional to frame size.
- Made scaling and box projection exact integer operations, including complete
  source-frame edges.
- Fixed one-pixel connected components and contrast percentiles on working
  frames smaller than 25 pixels.
- Area-scale masks consistently with area-scaled frames so an unmasked source
  pixel cannot be hidden by a masked sample point.
- Reject non-finite or out-of-range thresholds, unsafe geometry, and allocation
  sizes that cannot be represented safely.
- Avoid component labeling on skipped frames and saturate persistent-motion
  accounting instead of allowing long-running signed overflow.
- Harden `kmd-stream` argument and input parsing, including interrupted reads,
  truncated frames, 64-bit counters, and output failures.
- Add deterministic randomized coverage, sanitizer integration, stream tests,
  dependency tracking, strict warning-clean builds, and reproducible benchmark
  checksums.
- Remove the unnecessary `libm` link dependency.

## 0.1.0

- Initial public release.
