/*
 * Cost per frame, which is the number that decides whether motion can be
 * the gate in front of everything else.
 *
 * Two questions:
 *   - what does detect_height buy?  It is the largest lever in the module.
 *   - does improve_contrast earn its cost?  It is the most expensive
 *     stage after the dilate, and it is on by default.
 *
 * Reported per camera and per seven cameras at 10 fps, because "0.8 ms a
 * frame" means nothing until it is "5% of a core for the fleet".
 */

#include "kilix_motion_detect.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

enum { W = 640, H = 360, FRAMES = 300, WARMUP = 30 };

static unsigned seed = 1u;

static unsigned next_random(void)
{
    seed = seed * 1103515245u + 12345u;
    return seed >> 16;
}

/* Structured background plus noise, so contrast normalisation sees the
 * dynamic range a real camera has rather than a flat field. */
static void scene(uint8_t *frame, int offset)
{
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int value = 60 + (x * 120) / W + (y * 50) / H;

            if (((x / 64) + (y / 48)) % 3 == 0) {
                value -= 40;
            }
            value += (int)(next_random() & 7u);
            if (value < 0) { value = 0; }
            if (value > 255) { value = 255; }
            frame[(size_t)y * W + (size_t)x] = (uint8_t)value;
        }
    }
    for (int y = 150; y < 220; y++) {
        for (int x = offset; x < offset + 50 && x < W; x++) {
            if (x < 0) { continue; }
            frame[(size_t)y * W + (size_t)x] = 230u;
        }
    }
}

static double now_ms(void)
{
    struct timespec now;

    (void)clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec * 1000.0 + (double)now.tv_nsec / 1e6;
}

static double run(int detect_height, bool contrast, bool average,
                  uint8_t *frame)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    kmd_box boxes[64];
    double start;

    kmd_config_init(&config);
    config.width = W;
    config.height = H;
    config.detect_height = detect_height;
    config.improve_contrast = contrast;
    config.area_average = average;
    if (!kmd_detector_create(&detector, &config)) {
        return -1.0;
    }
    for (int i = 0; i < WARMUP; i++) {
        scene(frame, 100);
        (void)kmd_detect(detector, frame, boxes, 64u, NULL);
    }
    start = now_ms();
    for (int i = 0; i < FRAMES; i++) {
        scene(frame, 100 + (i % 40));
        (void)kmd_detect(detector, frame, boxes, 64u, NULL);
    }
    {
        const double elapsed = now_ms() - start;

        kmd_detector_free(detector);
        return elapsed / (double)FRAMES;
    }
}

int
main(void)
{
    static const int heights[] = {360, 180, 100, 64};
    uint8_t *frame = malloc((size_t)W * H);
    double baseline = 0.0;

    if (frame == NULL) {
        return 1;
    }
    (void)printf("source %dx%d, %d frames after %d warm-up\n\n",
                 W, H, FRAMES, WARMUP);
    (void)printf("%-8s %-9s %-8s %10s %9s %16s\n", "height",
                 "contrast", "scaling", "ms/frame", "vs 360p",
                 "seven cams @10fps");
    for (size_t i = 0u; i < sizeof(heights) / sizeof(heights[0]); i++) {
        for (int pass = 0; pass < 3; pass++) {
            const bool contrast = pass != 1;
            const bool average = pass != 2;
            const double ms = run(heights[i], contrast, average, frame);
            char label[32];

            if (ms < 0.0) {
                continue;
            }
            if (i == 0u && pass == 0) {
                baseline = ms;
            }
            (void)snprintf(label, sizeof(label), "%d", heights[i]);
            (void)printf("%-8s %-9s %-8s %10.3f %8.1fx %15.1f%%\n",
                         pass == 0 ? label : "", contrast ? "on" : "off",
                         average ? "area" : "point",
                         ms, baseline / ms, ms * 7.0);
        }
    }
    (void)printf("\nPercentages are of one core: ms/frame x fps / 10.\n");
    free(frame);
    return 0;
}
