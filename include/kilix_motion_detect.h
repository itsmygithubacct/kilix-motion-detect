#ifndef KILIX_MOTION_DETECT_H
#define KILIX_MOTION_DETECT_H

/*
 * Frame-difference motion detection: frames in, boxes out.
 *
 * This exists because motion is the cheap gate in front of expensive
 * work.  Object detection costs tens of milliseconds a frame; deciding
 * whether anything moved costs well under one.  Running the second only
 * where the first found something is what makes a multi-camera pipeline
 * fit on a small machine, and it is the reason a recorder can watch seven
 * cameras on hardware that could not run a model against one.
 *
 * The scope is deliberately narrow.  It reports where pixels changed and
 * nothing about what changed them: no classification, no tracking, no
 * grouping boxes into detector-sized regions.  Those depend on the
 * consumer - a detector wants square crops at its input resolution, a UI
 * overlay wants the boxes as they are - so they belong to the consumer.
 *
 * The only build dependency is a C11 toolchain.  No image library or workers.
 * kmd_detect() performs no allocation; detector creation and installing a
 * first mask may allocate.  A caller supplies pixels and gets integers back.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KILIX_MOTION_DETECT_VERSION_MAJOR 0
#define KILIX_MOTION_DETECT_VERSION_MINOR 2
#define KILIX_MOTION_DETECT_VERSION_PATCH 0

/*
 * Input pixel layout.
 *
 * Motion only reads luma, so grayscale input is the cheapest thing to
 * hand over.  Packed formats are accepted because the frame usually
 * already exists in one - a decoder feeding both a detector and a display
 * has to produce colour anyway, and converting it twice to save a byte
 * per pixel here would cost more than it saves.
 */
typedef enum kmd_pixfmt {
    KMD_PIXFMT_GRAY = 0,   /* 1 byte per pixel */
    KMD_PIXFMT_RGBA,       /* 4 bytes, byte order R,G,B,A */
    KMD_PIXFMT_BGRA        /* 4 bytes, byte order B,G,R,A */
} kmd_pixfmt;

typedef struct kmd_box {
    int x0;
    int y0;
    int x1;   /* exclusive */
    int y1;   /* exclusive */
} kmd_box;

typedef struct kmd_config {
    /* Source frame geometry.  Required. */
    int width;
    int height;
    kmd_pixfmt pixfmt;

    /*
     * Height to difference at.  The source is scaled down to this before
     * anything else, and every threshold below is expressed in that
     * smaller frame.
     *
     * This is the single largest cost lever in the module and it is
     * nearly free: differencing a 640x360 frame costs roughly eight times
     * what differencing it at 100 pixels tall does, while moving the
     * smallest detectable object from about four source pixels to about
     * six.  Nothing anyone cares about is six pixels.
     *
     * Default 100, which is also what Frigate uses.  Zero or a value at least
     * as large as the source height means "do not scale", which is almost
     * never what you want.
     */
    int detect_height;

    /*
     * Per-pixel difference, 1-255, above which a pixel counts as changed.
     * Default 30.  Lower finds subtler movement and more sensor noise;
     * IR footage is noisy enough that lowering this is usually the wrong
     * lever and detect_height is the right one.
     */
    int threshold;

    /*
     * Smallest connected blob, in scaled pixels, that becomes a box.
     * Default 10.
     *
     * Measured after dilation, which makes the effective floor larger
     * than it looks: an isolated changed pixel dilates to nine, so the
     * default rejects exactly one stray pixel and accepts two adjacent
     * ones.  This is the knob to raise when a camera reports insects,
     * rain or snow, and the one to leave alone otherwise.
     */
    int contour_area;

    /*
     * How fast the background model follows the scene, per frame.
     * Must be finite and in (0, 1].  Default 0.01: a change persists as
     * motion for about a hundred frames before becoming the new background.
     * Raise it and a person standing still vanishes sooner; lower it and a
     * moved chair reports motion for minutes.
     */
    float frame_alpha;

    /*
     * Normalise contrast before differencing, against a rolling window of
     * recent frames rather than the current one.
     *
     * The window is the point: normalising per frame makes a lighting
     * change look like whole-frame motion, which is the thing this is
     * supposed to suppress.  Default true.
     */
    bool improve_contrast;

    /*
     * Fraction of the frame changing that means "the scene changed"
     * rather than "something moved" - an IR cutover, a light switching
     * on, a camera being repointed.  Default 0.8.
     *
     * Must be finite and in [0, 1].  Crossing it forces the background model
     * to re-converge and sets `scene_changed` on the result.  Boxes are still
     * reported: the frame is flagged, not hidden.
     */
    float lightning_threshold;

    /*
     * Fraction of the frame changing above which boxes are suppressed
     * entirely and the frame reports `skipped`.
     *
     * Must be finite and in [0, 1].  Disabled by default (0), and that default
     * is deliberate.  Skipping is the only behaviour here that can hide
     * something real: a camera panning across a person reports no motion at
     * all, and a recorder gated on motion therefore stores nothing.  It saves
     * work during storms and IR flapping, which is a fair trade to make
     * knowingly and a poor one to inherit.
     */
    float skip_threshold;

    /*
     * Average the source pixels covering each scaled pixel, instead of
     * sampling one of them.
     *
     * This is not a quality nicety, it is what makes a detailed scene
     * usable.  Point sampling means sub-pixel camera shake, or a single
     * noisy sensor pixel, changes *which* source pixel a scaled pixel
     * takes its value from - and on a cluttered scene the two neighbours
     * either side of a high-contrast edge differ by more than any
     * threshold worth setting.  A still workshop full of tools and cables
     * reported motion on every frame until this was turned on.
     *
     * Costs more: it reads every source pixel rather than one per output
     * pixel.  Default true, because a detector that fires constantly is
     * not cheaper in any sense that matters.
     */
    bool area_average;

    /*
     * Box blur radius applied before differencing, in scaled pixels.
     * Default 1.  Zero disables it, which roughly doubles the number of
     * single-pixel sensor-noise blobs on IR footage.  Values larger than a
     * working dimension are clamped to that dimension's complete extent, so
     * work remains proportional to the frame rather than to the radius.
     */
    int blur_radius;
} kmd_config;

/* Fill with the defaults documented above. */
void kmd_config_init(kmd_config *config);

typedef struct kmd_detector kmd_detector;

/*
 * Allocate a detector.  All per-frame working buffers are allocated here, so
 * kmd_detect() cannot fail for want of memory and can be called from a loop
 * that must not stall.  Invalid, non-finite, unrepresentable, or unsafe-size
 * configuration is rejected and `*detector` is left NULL.
 */
bool kmd_detector_create(kmd_detector **detector, const kmd_config *config);
void kmd_detector_free(kmd_detector *detector);

/*
 * Ignore parts of the frame - a tree line, a road, a burnt-in timestamp.
 *
 * `mask` is one byte per source pixel, `width * height` of them: zero
 * ignores a pixel, anything else keeps it.  It is copied, so the caller's
 * buffer need not outlive the call.  NULL clears any existing mask.
 *
 * When area downscaling is active, a working cell remains active if any
 * source pixel contributing to it is active.  This matches area-resized mask
 * behavior and prevents a zero at one sample point from hiding other unmasked
 * pixels in the cell.  Mask boundaries therefore have working-cell
 * granularity; mask the complete source footprint of a cell to suppress it.
 *
 * A bitmap rather than polygons on purpose.  Rasterising a polygon is a
 * geometry problem, this module is arithmetic on pixels, and a caller
 * that has polygons already has something that can draw them.  It also
 * keeps the module free of dependencies.
 *
 * Masking happens after contrast normalisation and before differencing,
 * matching Frigate.  Masking first would let a bright masked region drag
 * the contrast window around and change results elsewhere in the frame.
 *
 * Changing the mask resets the background model: the masked-out pixels
 * are pinned to a constant, so the old model no longer describes the
 * frame being differenced.
 */
bool kmd_detector_set_mask(
    kmd_detector *detector, const uint8_t *mask, size_t size);

typedef struct kmd_result {
    /* Boxes written, which may be fewer than found if capacity ran out. */
    size_t count;
    /* Boxes found before capacity was applied. */
    size_t found;

    /* Fraction of the scaled frame that changed, 0-1. */
    float motion_fraction;

    /*
     * The background model has not converged yet, so boxes are unreliable
     * and a consumer should not act on them.  True for the first frames
     * after creation, after a reset, and after a scene change.
     */
    bool calibrating;

    /* motion_fraction crossed lightning_threshold: treat as a scene
     * change rather than as movement. */
    bool scene_changed;

    /* motion_fraction crossed skip_threshold, so boxes were suppressed.
     * Only ever true when skip_threshold is enabled. */
    bool skipped;
} kmd_result;

/*
 * Difference one frame against the background model and report where it
 * changed.
 *
 * `frame` is exactly width * height tightly packed pixels in the configured
 * format; because the pointer carries no size, the caller owns that bound.
 * Boxes are written in source coordinates - not the scaled ones the module
 * works in - so a caller never has to know detect_height.
 *
 * Returns the number of boxes written.  `result` may be NULL, but the
 * calibrating flag is worth reading: acting on boxes before the model has
 * converged produces a burst of false motion at every startup.
 *
 * Frames must arrive in order.  The model is a running average, so
 * feeding unrelated frames interleaved produces motion between them.
 * A detector is not internally synchronized; distinct detectors may be used
 * concurrently, while calls touching one detector require external ordering.
 */
size_t kmd_detect(
    kmd_detector *detector,
    const uint8_t *frame,
    kmd_box *boxes,
    size_t capacity,
    kmd_result *result);

/* Discard the background model; the next frames recalibrate.  Worth
 * calling when the stream restarts, since the scene may have changed
 * while it was down. */
void kmd_detector_reset(kmd_detector *detector);

/* The scaled geometry the module works in, for a caller that wants to
 * visualise the intermediate stages or size its own buffers. */
int kmd_detector_scaled_width(const kmd_detector *detector);
int kmd_detector_scaled_height(const kmd_detector *detector);

#ifdef __cplusplus
}
#endif

#endif /* KILIX_MOTION_DETECT_H */
