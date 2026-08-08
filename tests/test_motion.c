#include "kilix_motion_detect.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            (void)fprintf(stderr, "%s:%d: check failed: %s\n",                \
                          __FILE__, __LINE__, #condition);                    \
            return false;                                                     \
        }                                                                     \
    } while (false)

enum { W = 320, H = 180 };

/* A deterministic noise source, so a failure reproduces exactly rather
 * than one run in twenty. */
static unsigned seed = 1u;

static unsigned next_random(void)
{
    seed = seed * 1103515245u + 12345u;
    return seed >> 16;
}

/*
 * A scene with sensor-like noise, plus an optional brighter rectangle
 * standing in for a subject.
 *
 * The background has real structure - a gradient and a few fixed blocks -
 * rather than being flat grey, and that is not decoration.  Contrast
 * normalisation stretches whatever range the frame has to fill 0-255, so
 * a flat scene spanning seven grey levels has its sensor noise amplified
 * about thirty-six times and reads as continuous motion.  Real camera
 * frames span most of the range; a flat synthetic one is not a simpler
 * test case, it is a different and much harder one.
 */
static void scene(uint8_t *frame, int bx, int by, int bw, int bh,
                  uint8_t base, uint8_t subject)
{
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            /* gradient across the frame, plus fixed darker blocks */
            int value = base + (x * 90) / W + (y * 40) / H;

            if (((x / 40) + (y / 30)) % 3 == 0) {
                value -= 45;
            }
            value += (int)(next_random() & 7u);
            if (value < 0) { value = 0; }
            if (value > 255) { value = 255; }
            frame[(size_t)y * W + (size_t)x] = (uint8_t)value;
        }
    }
    for (int y = by; y < by + bh && y < H; y++) {
        for (int x = bx; x < bx + bw && x < W; x++) {
            if (x < 0 || y < 0) {
                continue;
            }
            frame[(size_t)y * W + (size_t)x] = subject;
        }
    }
}

/* Run the detector until it says it has converged, on an empty scene. */
static bool settle(kmd_detector *detector, uint8_t *frame, int frames)
{
    kmd_result result;

    for (int i = 0; i < frames; i++) {
        scene(frame, 0, 0, 0, 0, 90u, 0u);
        (void)kmd_detect(detector, frame, NULL, 0u, &result);
    }
    return !result.calibrating;
}

static bool
test_config_defaults_and_rejections(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;

    kmd_config_init(&config);
    CHECK(config.detect_height == 100);
    CHECK(config.threshold == 30);
    CHECK(config.contour_area == 10);
    /* Off by default, deliberately: skipping is the only behaviour that
     * can hide something real. */
    CHECK(config.skip_threshold == 0.0f);
    CHECK(config.improve_contrast);

    CHECK(!kmd_detector_create(NULL, &config));
    CHECK(!kmd_detector_create(&detector, NULL));
    CHECK(!kmd_detector_create(&detector, &config));   /* no geometry */

    config.width = W;
    config.height = H;
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(detector != NULL);
    /* Scaled to the working height, aspect preserved. */
    CHECK(kmd_detector_scaled_height(detector) == 100);
    CHECK(kmd_detector_scaled_width(detector) == 177);
    kmd_detector_free(detector);

    config.threshold = 0;
    CHECK(!kmd_detector_create(&detector, &config));
    config.threshold = 300;
    CHECK(!kmd_detector_create(&detector, &config));
    config.threshold = 30;
    config.contour_area = 0;
    CHECK(!kmd_detector_create(&detector, &config));
    config.contour_area = 10;
    config.frame_alpha = 0.0f;
    CHECK(!kmd_detector_create(&detector, &config));
    config.frame_alpha = 2.0f;
    CHECK(!kmd_detector_create(&detector, &config));
    config.frame_alpha = 0.01f;
    config.blur_radius = -1;
    CHECK(!kmd_detector_create(&detector, &config));
    config.blur_radius = 1;
    config.pixfmt = (kmd_pixfmt)99;
    CHECK(!kmd_detector_create(&detector, &config));

    /* NULL frame and NULL detector are answered, not crashed on. */
    CHECK(kmd_detect(NULL, NULL, NULL, 0u, NULL) == 0u);
    return true;
}

/*
 * The measurement that matters most.  Motion is the gate in front of
 * everything expensive, so a detector that fires on a still scene costs
 * more than one that misses a subject.
 */
static bool
test_a_still_scene_reports_no_motion(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_result result;
    int noisy_frames = 0;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));

    for (int i = 0; i < 200; i++) {
        scene(frame, 0, 0, 0, 0, 90u, 0u);
        if (kmd_detect(detector, frame, NULL, 0u, &result) > 0u) {
            noisy_frames++;
        }
    }
    /* Sensor noise alone must not produce boxes. */
    CHECK(noisy_frames == 0);
    CHECK(!result.calibrating);
    CHECK(result.motion_fraction < 0.01f);

    kmd_detector_free(detector);
    free(frame);
    return true;
}

static bool
test_a_moving_subject_is_found(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_box boxes[32];
    kmd_result result;
    size_t count;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));

    /* A 40x50 subject appears at (100, 60). */
    scene(frame, 100, 60, 40, 50, 90u, 220u);
    count = kmd_detect(detector, frame, boxes, 32u, &result);
    CHECK(count >= 1u);
    CHECK(result.found >= 1u);

    /* Boxes come back in source coordinates, not the scaled ones the
     * module works in, so a caller never has to know detect_height. */
    {
        bool covered = false;

        for (size_t i = 0u; i < count; i++) {
            if (boxes[i].x0 <= 110 && boxes[i].x1 >= 130 &&
                boxes[i].y0 <= 70 && boxes[i].y1 >= 100) {
                covered = true;
            }
            CHECK(boxes[i].x0 >= 0 && boxes[i].y0 >= 0);
            CHECK(boxes[i].x1 <= W && boxes[i].y1 <= H);
            CHECK(boxes[i].x1 > boxes[i].x0);
            CHECK(boxes[i].y1 > boxes[i].y0);
        }
        CHECK(covered);
    }
    kmd_detector_free(detector);
    free(frame);
    return true;
}

/*
 * contour_area is measured on the dilated image, matching Frigate, which
 * makes its effective floor larger than it looks: a single changed pixel
 * dilates to nine, so the default of 10 rejects exactly one isolated
 * pixel and accepts two adjacent ones.
 *
 * Tested 1:1 with blur off, so "pixel" means the same thing in the test
 * as in the module and the boundary is exact rather than approximate.
 */
static bool
test_contour_area_counts_dilated_pixels(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_result result;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    config.detect_height = 0;    /* 1:1, so source and scaled agree */
    config.blur_radius = 0;      /* a blur would smear one pixel away */

    /* One pixel dilates to nine, below the default of ten. */
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));
    scene(frame, 150, 90, 1, 1, 90u, 240u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.found == 0u);
    kmd_detector_free(detector);

    /* Two by two dilates to sixteen, above it. */
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));
    scene(frame, 150, 90, 2, 2, 90u, 240u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.found >= 1u);
    kmd_detector_free(detector);

    /* Raising contour_area rejects it again, which is the knob a caller
     * reaches for when a camera reports insects and rain. */
    config.contour_area = 64;
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));
    scene(frame, 150, 90, 2, 2, 90u, 240u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.found == 0u);
    scene(frame, 150, 90, 30, 30, 90u, 240u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.found >= 1u);
    kmd_detector_free(detector);

    free(frame);
    return true;
}

static bool
test_mask_suppresses_its_region(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    uint8_t *mask = malloc((size_t)W * H);
    kmd_result result;

    CHECK(frame != NULL && mask != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    CHECK(kmd_detector_create(&detector, &config));

    /* Ignore the left half. */
    memset(mask, 1, (size_t)W * H);
    for (int y = 0; y < H; y++) {
        memset(mask + (size_t)y * W, 0, (size_t)W / 2u);
    }
    CHECK(!kmd_detector_set_mask(detector, mask, 10u));   /* wrong size */
    CHECK(kmd_detector_set_mask(detector, mask, (size_t)W * H));
    CHECK(settle(detector, frame, 60));

    /* A subject entirely inside the masked half is not reported... */
    scene(frame, 20, 60, 60, 60, 90u, 220u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.found == 0u);

    /* ...while the same subject on the visible side is. */
    scene(frame, 200, 60, 60, 60, 90u, 220u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.found >= 1u);

    /* Clearing the mask is allowed and resets the model. */
    CHECK(kmd_detector_set_mask(detector, NULL, 0u));
    kmd_detector_free(detector);
    free(frame);
    free(mask);
    return true;
}

/*
 * A whole-frame lighting change is flagged, never hidden.  Hiding it is
 * what skip_threshold does, and that is opt-in for exactly this reason:
 * a consumer that never sees the frame cannot decide about it.
 */
static bool
test_scene_change_is_reported_not_hidden(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_result result;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    /* Contrast normalisation exists to absorb lighting changes, which is
     * the opposite of what this test wants to trigger. */
    config.improve_contrast = false;
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));

    /* Everything gets much brighter at once: an IR cutover. */
    scene(frame, 0, 0, 0, 0, 240u, 0u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.motion_fraction > 0.8f);
    CHECK(result.scene_changed);
    CHECK(result.calibrating);         /* the model must re-converge */
    CHECK(!result.skipped);            /* skipping is off by default */
    CHECK(result.found > 0u);          /* and the boxes still arrive */

    kmd_detector_free(detector);
    free(frame);
    return true;
}

static bool
test_skip_threshold_when_enabled(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_box boxes[8];
    kmd_result result;
    size_t count;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    config.improve_contrast = false;
    config.skip_threshold = 0.5f;
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));

    scene(frame, 0, 0, 0, 0, 240u, 0u);
    count = kmd_detect(detector, frame, boxes, 8u, &result);
    CHECK(result.skipped);
    CHECK(count == 0u);
    CHECK(result.count == 0u);
    CHECK(result.found == 0u);

    /* The model must follow the new condition, or every later frame skips
     * too and the camera goes permanently blind. */
    {
        bool recovered = false;

        for (int i = 0; i < 60; i++) {
            scene(frame, 0, 0, 0, 0, 240u, 0u);
            (void)kmd_detect(detector, frame, NULL, 0u, &result);
            if (!result.skipped) {
                recovered = true;
                break;
            }
        }
        CHECK(recovered);
    }
    kmd_detector_free(detector);
    free(frame);
    return true;
}

/* The same content in three layouts must produce the same answer, or a
 * caller's choice of decode format silently changes detection. */
static bool
test_pixel_formats_agree(void)
{
    kmd_config config;
    kmd_detector *gray_detector = NULL;
    kmd_detector *rgba_detector = NULL;
    kmd_detector *bgra_detector = NULL;
    uint8_t *gray = malloc((size_t)W * H);
    uint8_t *rgba = malloc((size_t)W * H * 4u);
    uint8_t *bgra = malloc((size_t)W * H * 4u);
    kmd_result gray_result;
    kmd_result rgba_result;
    kmd_result bgra_result;

    CHECK(gray != NULL && rgba != NULL && bgra != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    config.pixfmt = KMD_PIXFMT_GRAY;
    CHECK(kmd_detector_create(&gray_detector, &config));
    config.pixfmt = KMD_PIXFMT_RGBA;
    CHECK(kmd_detector_create(&rgba_detector, &config));
    config.pixfmt = KMD_PIXFMT_BGRA;
    CHECK(kmd_detector_create(&bgra_detector, &config));

    for (int i = 0; i < 70; i++) {
        const bool subject = i == 69;

        seed = 7u;
        scene(gray, subject ? 100 : 0, 60, subject ? 40 : 0, 50, 90u, 220u);
        /* Neutral grey, so luma equals the source byte in every layout. */
        for (size_t p = 0u; p < (size_t)W * H; p++) {
            rgba[p * 4u + 0u] = gray[p];
            rgba[p * 4u + 1u] = gray[p];
            rgba[p * 4u + 2u] = gray[p];
            rgba[p * 4u + 3u] = 255u;
            bgra[p * 4u + 0u] = gray[p];
            bgra[p * 4u + 1u] = gray[p];
            bgra[p * 4u + 2u] = gray[p];
            bgra[p * 4u + 3u] = 255u;
        }
        (void)kmd_detect(gray_detector, gray, NULL, 0u, &gray_result);
        (void)kmd_detect(rgba_detector, rgba, NULL, 0u, &rgba_result);
        (void)kmd_detect(bgra_detector, bgra, NULL, 0u, &bgra_result);
    }
    CHECK(gray_result.found >= 1u);
    CHECK(gray_result.found == rgba_result.found);
    CHECK(gray_result.found == bgra_result.found);

    /*
     * And the channel order genuinely matters.  One buffer, read two
     * ways: as RGBA its first byte is red (luma weight 77), as BGRA it is
     * blue (weight 29).  If these agreed, the format field would be
     * decorative and a caller's decode choice would silently swap what
     * the detector considers bright.
     */
    {
        kmd_detector *as_rgba = NULL;
        kmd_detector *as_bgra = NULL;
        kmd_result rgba_view;
        kmd_result bgra_view;

        config.pixfmt = KMD_PIXFMT_RGBA;
        CHECK(kmd_detector_create(&as_rgba, &config));
        config.pixfmt = KMD_PIXFMT_BGRA;
        CHECK(kmd_detector_create(&as_bgra, &config));

        for (size_t p = 0u; p < (size_t)W * H; p++) {
            rgba[p * 4u + 0u] = 255u;
            rgba[p * 4u + 1u] = 0u;
            rgba[p * 4u + 2u] = 0u;
            rgba[p * 4u + 3u] = 255u;
        }
        (void)kmd_detect(as_rgba, rgba, NULL, 0u, &rgba_view);
        (void)kmd_detect(as_bgra, rgba, NULL, 0u, &bgra_view);
        /* Against a zeroed model, luma 76 clears the default threshold of
         * 30 and luma 28 does not. */
        CHECK(rgba_view.motion_fraction > 0.9f);
        CHECK(bgra_view.motion_fraction < 0.1f);
        kmd_detector_free(as_rgba);
        kmd_detector_free(as_bgra);
    }

    kmd_detector_free(gray_detector);
    kmd_detector_free(rgba_detector);
    kmd_detector_free(bgra_detector);
    free(gray);
    free(rgba);
    free(bgra);
    return true;
}

/*
 * A subject that stops moving must eventually become background - a
 * parked car cannot report motion forever - but not so fast that a person
 * walking slowly fades out mid-frame.
 */
static bool
test_a_stationary_subject_fades_into_background(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_result result;
    int frames_until_quiet = -1;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    config.frame_alpha = 0.05f;   /* faster than default, to keep the test short */
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));

    for (int i = 0; i < 400; i++) {
        scene(frame, 100, 60, 40, 50, 90u, 220u);
        (void)kmd_detect(detector, frame, NULL, 0u, &result);
        if (result.found == 0u) {
            frames_until_quiet = i;
            break;
        }
    }
    CHECK(frames_until_quiet > 0);
    /* Persistence: it must survive well past a single frame, or a slow
     * walker disappears halfway across. */
    CHECK(frames_until_quiet >= 10);
    kmd_detector_free(detector);
    free(frame);
    return true;
}

/* Capacity limits what is written, never what is counted, so a caller
 * with a small array still learns there was more. */
static bool
test_capacity_is_reported_not_hidden(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_box boxes[2];
    kmd_result result;
    size_t count;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(settle(detector, frame, 60));

    /* Four separated subjects. */
    scene(frame, 0, 0, 0, 0, 90u, 0u);
    for (int i = 0; i < 4; i++) {
        for (int y = 40; y < 80; y++) {
            for (int x = 20 + i * 70; x < 50 + i * 70 && x < W; x++) {
                frame[(size_t)y * W + (size_t)x] = 220u;
            }
        }
    }
    count = kmd_detect(detector, frame, boxes, 2u, &result);
    CHECK(count == 2u);
    CHECK(result.count == 2u);
    CHECK(result.found >= 3u);   /* more were found than fitted */

    /* A NULL array still counts. */
    kmd_detector_reset(detector);
    CHECK(settle(detector, frame, 60));
    scene(frame, 100, 60, 40, 50, 90u, 220u);
    CHECK(kmd_detect(detector, frame, NULL, 0u, &result) == 0u);
    CHECK(result.found >= 1u);
    kmd_detector_free(detector);
    free(frame);
    return true;
}

/* Boxes are unreliable until the model converges, and the flag is how a
 * consumer knows not to act on the startup burst. */
static bool
test_calibration_is_reported(void)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    uint8_t *frame = malloc((size_t)W * H);
    kmd_result result;

    CHECK(frame != NULL);
    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    CHECK(kmd_detector_create(&detector, &config));

    scene(frame, 0, 0, 0, 0, 90u, 0u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.calibrating);            /* first frame, nothing learned */

    CHECK(settle(detector, frame, 60));   /* converges on a quiet scene */

    kmd_detector_reset(detector);
    scene(frame, 0, 0, 0, 0, 90u, 0u);
    (void)kmd_detect(detector, frame, NULL, 0u, &result);
    CHECK(result.calibrating);            /* reset puts it back */

    kmd_detector_free(detector);
    free(frame);
    return true;
}

typedef bool (*test_function)(void);

typedef struct test_case {
    const char *name;
    test_function function;
} test_case;

int
main(void)
{
    static const test_case tests[] = {
        {"config defaults and rejections", test_config_defaults_and_rejections},
        {"a still scene reports no motion",
         test_a_still_scene_reports_no_motion},
        {"a moving subject is found", test_a_moving_subject_is_found},
        {"contour area counts dilated pixels",
         test_contour_area_counts_dilated_pixels},
        {"mask suppresses its region", test_mask_suppresses_its_region},
        {"scene change is reported not hidden",
         test_scene_change_is_reported_not_hidden},
        {"skip threshold when enabled", test_skip_threshold_when_enabled},
        {"pixel formats agree", test_pixel_formats_agree},
        {"a stationary subject fades into background",
         test_a_stationary_subject_fades_into_background},
        {"capacity is reported not hidden",
         test_capacity_is_reported_not_hidden},
        {"calibration is reported", test_calibration_is_reported}
    };
    size_t passed = 0u;

    for (size_t index = 0u; index < sizeof(tests) / sizeof(tests[0]); ++index) {
        const bool ok = tests[index].function();

        (void)printf("%s %s\n", ok ? "ok" : "not ok", tests[index].name);
        if (!ok) {
            return 1;
        }
        ++passed;
    }
    (void)printf("%zu tests passed\n", passed);
    return 0;
}
