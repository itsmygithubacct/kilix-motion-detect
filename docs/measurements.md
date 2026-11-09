# Cost and accuracy measurements

What a frame costs at each working size, and the false-positive and recall measurements on real camera footage. Moved from the README.

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
