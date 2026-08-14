/*
 * Frame differencing against a running background model.
 *
 * The pipeline, in order, and each stage is there for a reason that shows
 * up on real camera footage rather than in theory:
 *
 *   1. scale down      the largest cost lever by far, and nearly free in
 *                      sensitivity - see detect_height in the header
 *   2. contrast        normalised against a rolling window of recent
 *                      frames, so a lighting change does not read as
 *                      whole-frame motion
 *   3. mask            after contrast, so a masked region cannot drag the
 *                      window around and change results elsewhere
 *   4. blur            kills isolated sensor-noise pixels, which IR
 *                      footage produces in quantity
 *   5. difference      against the model, threshold, dilate
 *   6. label           connected components, each big enough one a box
 *   7. accumulate      fold the frame into the model, but only after the
 *                      difference has persisted, so a single frame of
 *                      noise does not become the background
 *
 * This follows Frigate's ImprovedMotionDetector, which is the design that
 * has actually been run against thousands of cameras.  Two things are
 * ours: it is C with no OpenCV, so the connected-component pass is a
 * two-pass union-find rather than a library call; and masking is a
 * caller-supplied bitmap rather than polygons.
 */

#include "kilix_motion_detect.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Frames of min/max history the contrast window averages over.  Long
 * enough that one bright frame cannot move it, short enough to follow
 * dusk. */
#define CONTRAST_HISTORY 50

/* Consecutive frames of motion before the model starts following the
 * scene.  Without it a single noisy frame is averaged in as background
 * and the real object that follows it is differenced away. */
#define PERSIST_FRAMES 10

/* Calibration ends when the frame is this quiet: little enough changed,
 * few enough separate blobs. */
#define CALIBRATION_MOTION 0.05f
#define CALIBRATION_BOXES 4

struct kmd_detector {
    kmd_config config;

    int scaled_width;
    int scaled_height;
    size_t scaled_pixels;

    uint8_t *gray;        /* scaled luma */
    uint8_t *blurred;     /* blur scratch */
    uint8_t *binary;      /* thresholded */
    uint8_t *dilated;
    float *model;         /* background, one float per scaled pixel */
    uint8_t *mask;        /* scaled, 0 = ignore */
    bool has_mask;

    int *labels;
    int *parent;
    int *area;
    kmd_box *extents;

    uint8_t contrast_low[CONTRAST_HISTORY];
    uint8_t contrast_high[CONTRAST_HISTORY];
    int contrast_index;
    int contrast_filled;

    bool calibrating;
    int motion_frames;
};

void kmd_config_init(kmd_config *config)
{
    if (config == NULL) {
        return;
    }
    config->width = 0;
    config->height = 0;
    config->pixfmt = KMD_PIXFMT_GRAY;
    config->detect_height = 100;
    config->threshold = 30;
    config->contour_area = 10;
    config->frame_alpha = 0.01f;
    config->improve_contrast = true;
    config->lightning_threshold = 0.8f;
    config->skip_threshold = 0.0f;   /* off; see the header for why */
    config->blur_radius = 1;
    config->area_average = true;
}

/* Rec.601 luma.  Integer weights summing to 256 so the divide is a shift;
 * motion cares about relative change, not colorimetric accuracy. */
static uint8_t luma(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint8_t)(((unsigned)r * 77u + (unsigned)g * 150u +
                      (unsigned)b * 29u) >> 8);
}

/*
 * Scale to the working size while converting to luma, in one pass.
 *
 * Two ways to do it, and the difference is not cosmetic.
 *
 * Point sampling takes one source pixel per output pixel.  It is cheap,
 * and on a cluttered scene it is unusable: sub-pixel shake or one noisy
 * sensor pixel changes which source pixel is taken, and across a
 * high-contrast edge the two candidates differ by more than any sensible
 * threshold.  Measured on a still workshop full of tools and cables, it
 * reported motion on 100% of frames.
 *
 * Area averaging reads every source pixel covering the output pixel.  It
 * costs more and it is the default, because a gate that never closes
 * saves nothing downstream.
 */
static void downscale_luma(const kmd_detector *detector, const uint8_t *frame)
{
    const int source_width = detector->config.width;
    const int source_height = detector->config.height;
    const int width = detector->scaled_width;
    const int height = detector->scaled_height;
    const kmd_pixfmt format = detector->config.pixfmt;
    uint8_t *out = detector->gray;
    const int y_step = source_height / height;
    const int y_remainder = source_height % height;
    int source_y = 0;
    int64_t y_error = 0;

    for (int y = 0; y < height; y++) {
        const int x_step = source_width / width;
        const int x_remainder = source_width % width;
        int source_x = 0;
        int64_t x_error = 0;

        for (int x = 0; x < width; x++) {
            const size_t index = (size_t)source_y * (size_t)source_width +
                                 (size_t)source_x;

            if (format == KMD_PIXFMT_GRAY) {
                out[(size_t)y * (size_t)width + (size_t)x] = frame[index];
            } else {
                const uint8_t *pixel = frame + index * 4u;

                out[(size_t)y * (size_t)width + (size_t)x] =
                    format == KMD_PIXFMT_BGRA
                        ? luma(pixel[2], pixel[1], pixel[0])
                        : luma(pixel[0], pixel[1], pixel[2]);
            }
            source_x += x_step;
            x_error += x_remainder;
            if (x_error >= width) {
                source_x++;
                x_error -= width;
            }
        }
        source_y += y_step;
        y_error += y_remainder;
        if (y_error >= height) {
            source_y++;
            y_error -= height;
        }
    }
}

/* Area-averaging variant: every source pixel covered by an output pixel
 * contributes, so a shifted sample cannot swing the result. */
static void downscale_luma_average(
    const kmd_detector *detector, const uint8_t *frame)
{
    const int source_width = detector->config.width;
    const int source_height = detector->config.height;
    const int width = detector->scaled_width;
    const int height = detector->scaled_height;
    const kmd_pixfmt format = detector->config.pixfmt;
    uint8_t *out = detector->gray;
    const int y_step = source_height / height;
    const int y_remainder = source_height % height;
    int y0 = 0;
    int64_t y_error = 0;

    for (int y = 0; y < height; y++) {
        const int x_step = source_width / width;
        const int x_remainder = source_width % width;
        int y1 = y0 + y_step;
        int x0 = 0;
        int64_t x_error = 0;

        y_error += y_remainder;
        if (y_error >= height) {
            y1++;
            y_error -= height;
        }
        for (int x = 0; x < width; x++) {
            int x1 = x0 + x_step;
            uint64_t total = 0u;
            uint64_t taken;

            x_error += x_remainder;
            if (x_error >= width) {
                x1++;
                x_error -= width;
            }
            if (format == KMD_PIXFMT_GRAY) {
                for (int sy = y0; sy < y1; sy++) {
                    const uint8_t *pixel =
                        frame + (size_t)sy * (size_t)source_width + (size_t)x0;

                    for (int sx = x0; sx < x1; sx++) {
                        total += *pixel++;
                    }
                }
            } else {
                for (int sy = y0; sy < y1; sy++) {
                    const uint8_t *pixel =
                        frame + ((size_t)sy * (size_t)source_width +
                                 (size_t)x0) * 4u;

                    for (int sx = x0; sx < x1; sx++) {
                        total += format == KMD_PIXFMT_BGRA
                                     ? luma(pixel[2], pixel[1], pixel[0])
                                     : luma(pixel[0], pixel[1], pixel[2]);
                        pixel += 4;
                    }
                }
            }
            taken = (uint64_t)(unsigned)(x1 - x0) *
                    (uint64_t)(unsigned)(y1 - y0);
            out[(size_t)y * (size_t)width + (size_t)x] =
                (uint8_t)(total / taken);
            x0 = x1;
        }
        y0 = y1;
    }
}

/*
 * Stretch contrast against an average of recent frames' extremes.
 *
 * Averaging over a window is the whole point.  Normalising against the
 * current frame's own min and max makes every lighting change look like
 * whole-frame motion - exactly the event this is meant to absorb - and
 * makes a frame containing one bright headlight rescale everything else.
 *
 * The percentiles are 4th and 96th rather than min and max so a handful
 * of hot pixels cannot define the range.
 */
static void improve_contrast(kmd_detector *detector)
{
    const size_t count = detector->scaled_pixels;
    uint8_t *gray = detector->gray;
    size_t cumulative = 0u;
    int low = 0;
    int high = 255;
    unsigned bins[256];

    memset(bins, 0, sizeof(bins));
    for (size_t i = 0u; i < count; i++) {
        bins[gray[i]]++;
    }
    {
        const size_t low_target =
            count / 25u > 0u ? count / 25u : 1u;       /* 4th percentile */
        const size_t high_target = count - count / 25u; /* 96th */

        cumulative = 0u;
        for (int value = 0; value < 256; value++) {
            cumulative += bins[value];
            if (cumulative >= low_target) {
                low = value;
                break;
            }
        }
        cumulative = 0u;
        for (int value = 0; value < 256; value++) {
            cumulative += bins[value];
            if (cumulative >= high_target) {
                high = value;
                break;
            }
        }
    }
    if (low >= high) {
        /* A single-colour frame has no contrast to improve, and dividing
         * by the range would be a divide by zero. */
        return;
    }

    detector->contrast_low[detector->contrast_index] = (uint8_t)low;
    detector->contrast_high[detector->contrast_index] = (uint8_t)high;
    detector->contrast_index =
        (detector->contrast_index + 1) % CONTRAST_HISTORY;
    if (detector->contrast_filled < CONTRAST_HISTORY) {
        detector->contrast_filled++;
    }

    {
        unsigned low_sum = 0u;
        unsigned high_sum = 0u;
        int filled = detector->contrast_filled;
        float average_low;
        float average_high;
        float span;

        for (int i = 0; i < filled; i++) {
            low_sum += detector->contrast_low[i];
            high_sum += detector->contrast_high[i];
        }
        average_low = (float)low_sum / (float)filled;
        average_high = (float)high_sum / (float)filled;
        span = average_high - average_low;
        if (span < 1.0f) {
            return;
        }
        /* The stretch depends only on the 8-bit input value, so evaluate
         * the float expression once per value rather than once per pixel;
         * the table is filled by the same expression, so the output bytes
         * cannot change. */
        uint8_t lut[256];

        for (int v = 0; v < 256; v++) {
            float value = ((float)v - average_low) / span * 255.0f;

            if (value < 0.0f) {
                value = 0.0f;
            } else if (value > 255.0f) {
                value = 255.0f;
            }
            lut[v] = (uint8_t)value;
        }
        for (size_t i = 0u; i < count; i++) {
            gray[i] = lut[gray[i]];
        }
    }
}

/* Separable box blur.  Cheap, and the difference from a gaussian is not
 * measurable once the result is thresholded. */
static void blur(kmd_detector *detector)
{
    const size_t width = (size_t)detector->scaled_width;
    const size_t height = (size_t)detector->scaled_height;
    const int radius = detector->config.blur_radius;
    uint8_t *gray = detector->gray;
    uint8_t *scratch = detector->blurred;
    size_t horizontal_radius;
    size_t vertical_radius;
    uint64_t full;
    uint64_t magic;

    if (radius <= 0) {
        return;
    }
    horizontal_radius = (size_t)radius < width ? (size_t)radius : width - 1u;
    vertical_radius = (size_t)radius < height ? (size_t)radius : height - 1u;

    /*
     * Away from the clamped edges every window in a pass has the same
     * width d, so the per-pixel division is by a loop-invariant divisor
     * and becomes a fixed-point reciprocal multiply, which is worth
     * about a third of the default-configuration frame time.  With
     * m = 2^40 / d + 1, (n * m) >> 40 equals n / d exactly for every
     * window sum n <= 255 * d whenever d <= 65535, because
     * 255 * d * d < 2^40.  Edge windows and oversized divisors keep the
     * plain division.  The accumulator stays 64-bit: accepted geometries
     * exist (one row, millions of pixels wide, the radius clamped to the
     * width) whose edge window sums overflow 32 bits.
     */
    full = 2u * (uint64_t)horizontal_radius + 1u;
    magic = full <= 65535u ? (UINT64_C(1) << 40) / full + 1u : 0u;
    for (size_t y = 0u; y < height; y++) {
        const uint8_t *row = gray + y * width;
        uint8_t *out = scratch + y * width;
        uint64_t total = 0u;
        size_t taken = horizontal_radius + 1u;

        for (size_t sample = 0u; sample <= horizontal_radius; sample++) {
            total += row[sample];
        }
        for (size_t x = 0u; x < width; x++) {
            out[x] = taken == full && magic != 0u
                         ? (uint8_t)((total * magic) >> 40)
                         : (uint8_t)(total / taken);
            if (x >= horizontal_radius) {
                total -= row[x - horizontal_radius];
                taken--;
            }
            if (x + horizontal_radius + 1u < width) {
                total += row[x + horizontal_radius + 1u];
                taken++;
            }
        }
    }
    full = 2u * (uint64_t)vertical_radius + 1u;
    magic = full <= 65535u ? (UINT64_C(1) << 40) / full + 1u : 0u;
    for (size_t x = 0u; x < width; x++) {
        uint64_t total = 0u;
        size_t taken = vertical_radius + 1u;

        for (size_t sample = 0u; sample <= vertical_radius; sample++) {
            total += scratch[sample * width + x];
        }
        for (size_t y = 0u; y < height; y++) {
            gray[y * width + x] = taken == full && magic != 0u
                                      ? (uint8_t)((total * magic) >> 40)
                                      : (uint8_t)(total / taken);
            if (y >= vertical_radius) {
                total -= scratch[(y - vertical_radius) * width + x];
                taken--;
            }
            if (y + vertical_radius + 1u < height) {
                total += scratch[(y + vertical_radius + 1u) * width + x];
                taken++;
            }
        }
    }
}

/* A 3x3 maximum over a binary image.  Two one-dimensional passes produce
 * exactly the same neighbourhood as nine bounded probes per output pixel. */
static void dilate(kmd_detector *detector)
{
    const size_t width = (size_t)detector->scaled_width;
    const size_t height = (size_t)detector->scaled_height;
    const uint8_t *binary = detector->binary;
    uint8_t *horizontal = detector->blurred;
    uint8_t *out = detector->dilated;

    for (size_t y = 0u; y < height; y++) {
        const size_t row = y * width;

        if (width == 1u) {
            horizontal[row] = binary[row];
            continue;
        }
        horizontal[row] = (uint8_t)(binary[row] | binary[row + 1u]);
        for (size_t x = 1u; x + 1u < width; x++) {
            horizontal[row + x] =
                (uint8_t)(binary[row + x - 1u] | binary[row + x] |
                          binary[row + x + 1u]);
        }
        horizontal[row + width - 1u] =
            (uint8_t)(binary[row + width - 2u] | binary[row + width - 1u]);
    }

    if (height == 1u) {
        memcpy(out, horizontal, width);
        return;
    }
    for (size_t x = 0u; x < width; x++) {
        out[x] = (uint8_t)(horizontal[x] | horizontal[width + x]);
    }
    for (size_t y = 1u; y + 1u < height; y++) {
        const size_t row = y * width;

        for (size_t x = 0u; x < width; x++) {
            out[row + x] =
                (uint8_t)(horizontal[row - width + x] | horizontal[row + x] |
                          horizontal[row + width + x]);
        }
    }
    for (size_t x = 0u; x < width; x++) {
        const size_t last = (height - 1u) * width + x;

        out[last] = (uint8_t)(horizontal[last - width] | horizontal[last]);
    }
}

/* Union-find over the binary image.  This is the one stage Frigate gets
 * from OpenCV; without it, a moving object becomes a scatter of pixels
 * rather than one box. */
static int label_find(int *parent, int label)
{
    while (parent[label] != label) {
        parent[label] = parent[parent[label]];   /* path halving */
        label = parent[label];
    }
    return label;
}

static void label_union(int *parent, int a, int b)
{
    a = label_find(parent, a);
    b = label_find(parent, b);
    if (a == b) {
        return;
    }
    if (a < b) {
        parent[b] = a;
    } else {
        parent[a] = b;
    }
}

static int scale_coordinate(int coordinate, int source, int scaled)
{
    return (int)((int64_t)coordinate * (int64_t)source / (int64_t)scaled);
}

static size_t label_components(kmd_detector *detector, kmd_box *boxes,
                               size_t capacity, size_t *found_out)
{
    const int width = detector->scaled_width;
    const int height = detector->scaled_height;
    const uint8_t *binary = detector->dilated;
    int *labels = detector->labels;
    int *parent = detector->parent;
    int *area = detector->area;
    kmd_box *extents = detector->extents;
    const size_t limit = detector->scaled_pixels;
    int next = 1;
    size_t written = 0u;
    size_t found = 0u;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const size_t index = (size_t)y * (size_t)width + (size_t)x;
            int up;
            int left;

            if (binary[index] == 0u) {
                labels[index] = 0;
                continue;
            }
            up = y > 0 ? labels[index - (size_t)width] : 0;
            left = x > 0 ? labels[index - 1u] : 0;
            if (up == 0 && left == 0) {
                if ((size_t)next > limit) {
                    continue;   /* pathological; the frame is all noise */
                }
                parent[next] = next;
                labels[index] = next;
                next++;
            } else if (up != 0 && left != 0) {
                labels[index] = up < left ? up : left;
                if (up != left) {
                    label_union(parent, up, left);
                }
            } else {
                labels[index] = up != 0 ? up : left;
            }
        }
    }

    for (int i = 0; i < next; i++) {
        area[i] = 0;
        extents[i].x0 = width;
        extents[i].y0 = height;
        extents[i].x1 = -1;
        extents[i].y1 = -1;
    }
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const int label = labels[(size_t)y * (size_t)width + (size_t)x];
            int root;

            if (label == 0) {
                continue;
            }
            root = label_find(parent, label);
            area[root]++;
            if (x < extents[root].x0) { extents[root].x0 = x; }
            if (y < extents[root].y0) { extents[root].y0 = y; }
            if (x + 1 > extents[root].x1) { extents[root].x1 = x + 1; }
            if (y + 1 > extents[root].y1) { extents[root].y1 = y + 1; }
        }
    }

    /* Scale extents back to source coordinates, so a caller never has to
     * know what detect_height was. */
    for (int i = 1; i < next; i++) {
        if (area[i] < detector->config.contour_area) {
            continue;
        }
        found++;
        if (boxes == NULL || written >= capacity) {
            continue;
        }
        boxes[written].x0 =
            scale_coordinate(extents[i].x0, detector->config.width, width);
        boxes[written].y0 =
            scale_coordinate(extents[i].y0, detector->config.height, height);
        boxes[written].x1 =
            scale_coordinate(extents[i].x1, detector->config.width, width);
        boxes[written].y1 =
            scale_coordinate(extents[i].y1, detector->config.height, height);
        written++;
    }
    if (found_out != NULL) {
        *found_out = found;
    }
    return written;
}

bool kmd_detector_create(kmd_detector **out, const kmd_config *config)
{
    kmd_detector *detector;
    int scaled_width;
    int scaled_height;
    size_t source_pixels;
    size_t bytes_per_pixel;
    size_t pixels;

    if (out == NULL) {
        return false;
    }
    *out = NULL;
    if (config == NULL || config->width <= 0 || config->height <= 0) {
        return false;
    }
    if (config->pixfmt != KMD_PIXFMT_GRAY &&
        config->pixfmt != KMD_PIXFMT_RGBA &&
        config->pixfmt != KMD_PIXFMT_BGRA) {
        return false;
    }
    if (config->threshold < 1 || config->threshold > 255) {
        return false;
    }
    if (config->contour_area < 1 || config->blur_radius < 0) {
        return false;
    }
    if (!isfinite(config->frame_alpha) || config->frame_alpha <= 0.0f ||
        config->frame_alpha > 1.0f) {
        return false;
    }
    if (!isfinite(config->lightning_threshold) ||
        config->lightning_threshold < 0.0f ||
        config->lightning_threshold > 1.0f) {
        return false;
    }
    if (!isfinite(config->skip_threshold) || config->skip_threshold < 0.0f ||
        config->skip_threshold > 1.0f) {
        return false;
    }
    if (config->detect_height < 0) {
        return false;
    }

    if ((size_t)config->width > SIZE_MAX / (size_t)config->height) {
        return false;
    }
    source_pixels = (size_t)config->width * (size_t)config->height;
    bytes_per_pixel = config->pixfmt == KMD_PIXFMT_GRAY ? 1u : 4u;
    if (source_pixels > SIZE_MAX / bytes_per_pixel ||
        source_pixels > UINT64_MAX / UINT8_MAX) {
        return false;
    }

    scaled_height = config->detect_height > 0 &&
                            config->detect_height < config->height
                        ? config->detect_height
                        : config->height;
    scaled_width = (int)((int64_t)config->width * (int64_t)scaled_height /
                         (int64_t)config->height);
    if (scaled_width < 1) {
        scaled_width = 1;
    }
    pixels = (size_t)scaled_width * (size_t)scaled_height;
    if (pixels > (size_t)INT_MAX - 1u ||
        pixels > SIZE_MAX / sizeof(kmd_box) - 1u) {
        return false;
    }

    detector = calloc(1u, sizeof(*detector));
    if (detector == NULL) {
        return false;
    }
    detector->config = *config;
    detector->scaled_width = scaled_width;
    detector->scaled_height = scaled_height;
    detector->scaled_pixels = pixels;
    detector->calibrating = true;

    detector->gray = calloc(pixels, 1u);
    detector->blurred = calloc(pixels, 1u);
    detector->binary = calloc(pixels, 1u);
    detector->dilated = calloc(pixels, 1u);
    detector->model = calloc(pixels, sizeof(float));
    detector->labels = calloc(pixels, sizeof(int));
    detector->parent = calloc(pixels + 1u, sizeof(int));
    detector->area = calloc(pixels + 1u, sizeof(int));
    detector->extents = calloc(pixels + 1u, sizeof(kmd_box));
    if (detector->gray == NULL || detector->blurred == NULL ||
        detector->binary == NULL || detector->dilated == NULL ||
        detector->model == NULL || detector->labels == NULL ||
        detector->parent == NULL || detector->area == NULL ||
        detector->extents == NULL) {
        kmd_detector_free(detector);
        return false;
    }
    *out = detector;
    return true;
}

void kmd_detector_free(kmd_detector *detector)
{
    if (detector == NULL) {
        return;
    }
    free(detector->gray);
    free(detector->blurred);
    free(detector->binary);
    free(detector->dilated);
    free(detector->model);
    free(detector->mask);
    free(detector->labels);
    free(detector->parent);
    free(detector->area);
    free(detector->extents);
    free(detector);
}

bool kmd_detector_set_mask(
    kmd_detector *detector, const uint8_t *mask, size_t size)
{
    size_t expected;

    if (detector == NULL) {
        return false;
    }
    if (mask == NULL) {
        free(detector->mask);
        detector->mask = NULL;
        detector->has_mask = false;
        kmd_detector_reset(detector);
        return true;
    }
    expected = (size_t)detector->config.width * (size_t)detector->config.height;
    if (size != expected) {
        return false;
    }
    if (detector->mask == NULL) {
        detector->mask = calloc(detector->scaled_pixels, 1u);
        if (detector->mask == NULL) {
            return false;
        }
    }
    if (detector->scaled_width == detector->config.width &&
        detector->scaled_height == detector->config.height) {
        memcpy(detector->mask, mask, expected);
    } else if (detector->config.area_average) {
        /* INTER_AREA-style mask semantics: a scaled cell remains active if
         * any source pixel contributing to it is active.  Point-sampling the
         * mask while area-averaging the frame can otherwise discard motion
         * from unmasked pixels that share a cell with the sample point. */
        for (int y = 0; y < detector->scaled_height; y++) {
            const int y0 = (int)((int64_t)y * detector->config.height /
                                 detector->scaled_height);
            const int y1 = (int)((int64_t)(y + 1) * detector->config.height /
                                 detector->scaled_height);

            for (int x = 0; x < detector->scaled_width; x++) {
                const int x0 = (int)((int64_t)x * detector->config.width /
                                     detector->scaled_width);
                const int x1 =
                    (int)((int64_t)(x + 1) * detector->config.width /
                          detector->scaled_width);
                uint8_t keep = 0u;

                for (int sy = y0; sy < y1 && keep == 0u; sy++) {
                    for (int sx = x0; sx < x1; sx++) {
                        if (mask[(size_t)sy * (size_t)detector->config.width +
                                 (size_t)sx] != 0u) {
                            keep = 1u;
                            break;
                        }
                    }
                }
                detector->mask[(size_t)y * (size_t)detector->scaled_width +
                               (size_t)x] = keep;
            }
        }
    } else {
        /* Point-sampled frames retain point-sampled mask semantics. */
        for (int y = 0; y < detector->scaled_height; y++) {
            const int source_y = (int)((int64_t)y * detector->config.height /
                                       detector->scaled_height);

            for (int x = 0; x < detector->scaled_width; x++) {
                const int source_x =
                    (int)((int64_t)x * detector->config.width /
                          detector->scaled_width);

                detector->mask[(size_t)y * (size_t)detector->scaled_width +
                               (size_t)x] =
                    mask[(size_t)source_y * (size_t)detector->config.width +
                         (size_t)source_x];
            }
        }
    }
    detector->has_mask = true;
    /* The masked pixels are about to be pinned to a constant, so the old
     * model no longer describes the frame being differenced. */
    kmd_detector_reset(detector);
    return true;
}

void kmd_detector_reset(kmd_detector *detector)
{
    if (detector == NULL) {
        return;
    }
    memset(detector->model, 0, detector->scaled_pixels * sizeof(float));
    detector->calibrating = true;
    detector->motion_frames = 0;
    detector->contrast_index = 0;
    detector->contrast_filled = 0;
}

int kmd_detector_scaled_width(const kmd_detector *detector)
{
    return detector != NULL ? detector->scaled_width : 0;
}

int kmd_detector_scaled_height(const kmd_detector *detector)
{
    return detector != NULL ? detector->scaled_height : 0;
}

size_t kmd_detect(
    kmd_detector *detector,
    const uint8_t *frame,
    kmd_box *boxes,
    size_t capacity,
    kmd_result *result)
{
    const size_t pixels = detector != NULL ? detector->scaled_pixels : 0u;
    size_t changed = 0u;
    size_t written = 0u;
    size_t found = 0u;
    float fraction;

    if (result != NULL) {
        result->count = 0u;
        result->found = 0u;
        result->motion_fraction = 0.0f;
        result->calibrating = true;
        result->scene_changed = false;
        result->skipped = false;
    }
    if (detector == NULL || frame == NULL) {
        return 0u;
    }

    if (detector->config.area_average) {
        downscale_luma_average(detector, frame);
    } else {
        downscale_luma(detector, frame);
    }
    if (detector->config.improve_contrast) {
        improve_contrast(detector);
    }
    if (detector->has_mask) {
        /* Zero rather than skip: the model holds zero there too, so a
         * masked pixel differences to nothing without a branch in the
         * inner loop. */
        for (size_t i = 0u; i < pixels; i++) {
            if (detector->mask[i] == 0u) {
                detector->gray[i] = 0u;
            }
        }
    }
    blur(detector);

    for (size_t i = 0u; i < pixels; i++) {
        const int model = (int)(detector->model[i] + 0.5f);
        int difference = (int)detector->gray[i] - model;

        if (difference < 0) {
            difference = -difference;
        }
        if (difference > detector->config.threshold) {
            detector->binary[i] = 1u;
            changed++;
        } else {
            detector->binary[i] = 0u;
        }
    }

    fraction = pixels > 0u ? (float)changed / (float)pixels : 0.0f;

    /* A skipped frame has no boxes by definition.  Decide before dilation and
     * component labeling so a storm, camera pan, or IR cutover does not spend
     * time constructing results that will immediately be discarded. */
    if (detector->config.skip_threshold > 0.0f &&
        fraction > detector->config.skip_threshold) {
        detector->calibrating = true;
        if (result != NULL) {
            result->motion_fraction = fraction;
            result->calibrating = true;
            result->scene_changed =
                fraction > detector->config.lightning_threshold;
            result->skipped = true;
        }
        /* Still fold the frame in, or the model never catches up to the
         * condition that caused the skip and every later frame skips too. */
        for (size_t i = 0u; i < pixels; i++) {
            detector->model[i] = detector->model[i] * 0.8f +
                                 (float)detector->gray[i] * 0.2f;
        }
        return 0u;
    }

    /* Dilate, so a blob broken by a single sub-threshold pixel stays one
     * blob rather than becoming two boxes. */
    dilate(detector);
    written = label_components(detector, boxes, capacity, &found);

    /* Converged once the scene is quiet: little changed, few blobs. */
    if (fraction < CALIBRATION_MOTION && found <= (size_t)CALIBRATION_BOXES) {
        detector->calibrating = false;
    }

    if (result != NULL) {
        result->motion_fraction = fraction;
        result->calibrating = detector->calibrating;
    }

    /* A scene change is reported, never hidden: the model re-converges
     * but the boxes still go to the caller. */
    if (fraction > detector->config.lightning_threshold) {
        detector->calibrating = true;
        if (result != NULL) {
            result->scene_changed = true;
            result->calibrating = true;
        }
    }

    /*
     * Fold the frame into the model.
     *
     * Motion has to persist before it is absorbed.  Averaging in a single
     * frame of movement makes the object part of the background, and the
     * next frame differences it away - so a slow walk across the frame
     * would fade out halfway.  While calibrating the rate is much higher,
     * because converging fast is the whole job.
     */
    {
        const float alpha = detector->calibrating
                                ? 0.2f
                                : detector->config.frame_alpha;

        if (found > 0u) {
            if (detector->motion_frames < PERSIST_FRAMES) {
                detector->motion_frames++;
            }
            if (detector->motion_frames >= PERSIST_FRAMES) {
                for (size_t i = 0u; i < pixels; i++) {
                    detector->model[i] =
                        detector->model[i] * (1.0f - alpha) +
                        (float)detector->gray[i] * alpha;
                }
            }
        } else {
            detector->motion_frames = 0;
            for (size_t i = 0u; i < pixels; i++) {
                detector->model[i] = detector->model[i] * (1.0f - alpha) +
                                     (float)detector->gray[i] * alpha;
            }
        }
    }

    if (result != NULL) {
        result->count = written;
        result->found = found;
    }
    return written;
}
