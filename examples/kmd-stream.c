/*
 * Run the detector over raw frames on stdin and report what it found.
 *
 * Exists so the module can be measured against real camera footage rather
 * than against synthetic scenes, which is the only way to know its false
 * positive rate - the number that decides whether motion is usable as a
 * gate at all.
 *
 *   ffmpeg -i clip.mkv -f rawvideo -pix_fmt gray -vf scale=640:360 - \
 *     | kmd-stream 640 360 [detect_height] [contrast:0|1]
 *
 * Prints one line per frame with motion, and a summary to stderr.
 */

#include "kilix_motion_detect.h"

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef enum read_status {
    READ_FRAME,
    READ_END,
    READ_TRUNCATED,
    READ_ERROR
} read_status;

static bool parse_int(const char *text, int minimum, int maximum, int *value)
{
    char *end = NULL;
    long parsed;

    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed < minimum ||
        parsed > maximum) {
        return false;
    }
    *value = (int)parsed;
    return true;
}

static read_status read_full(uint8_t *buffer, size_t size)
{
    size_t total = 0u;

    while (total < size) {
        size_t request = size - total;
        ssize_t got;

        if (request > (size_t)SSIZE_MAX) {
            request = (size_t)SSIZE_MAX;
        }
        got = read(STDIN_FILENO, buffer + total, request);
        if (got > 0) {
            total += (size_t)got;
            continue;
        }
        if (got == 0) {
            return total == 0u ? READ_END : READ_TRUNCATED;
        }
        if (errno != EINTR) {
            return READ_ERROR;
        }
    }
    return READ_FRAME;
}

int
main(int argc, char **argv)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    kmd_box boxes[64];
    kmd_result result;
    uint8_t *frame;
    size_t frame_size;
    uint64_t frames = 0u;
    uint64_t motion_frames = 0u;
    uint64_t calibrating_frames = 0u;
    uint64_t scene_changes = 0u;
    uint64_t total_boxes = 0u;
    int parsed;

    if (argc < 3 || argc > 8) {
        (void)fprintf(stderr,
                      "usage: %s WIDTH HEIGHT [DETECT_HEIGHT] [CONTRAST 0|1] "
                      "[AREA_AVERAGE 0|1] [THRESHOLD] [CONTOUR_AREA]\n",
                      argv[0]);
        return 2;
    }
    kmd_config_init(&config);
    if (!parse_int(argv[1], 1, INT_MAX, &config.width) ||
        !parse_int(argv[2], 1, INT_MAX, &config.height)) {
        (void)fprintf(stderr, "WIDTH and HEIGHT must be positive integers\n");
        return 2;
    }
    config.pixfmt = KMD_PIXFMT_GRAY;
    if (argc > 3 &&
        !parse_int(argv[3], 0, INT_MAX, &config.detect_height)) {
        (void)fprintf(stderr, "DETECT_HEIGHT must be a non-negative integer\n");
        return 2;
    }
    if (argc > 4) {
        if (!parse_int(argv[4], 0, 1, &parsed)) {
            (void)fprintf(stderr, "CONTRAST must be 0 or 1\n");
            return 2;
        }
        config.improve_contrast = parsed != 0;
    }
    if (argc > 5) {
        if (!parse_int(argv[5], 0, 1, &parsed)) {
            (void)fprintf(stderr, "AREA_AVERAGE must be 0 or 1\n");
            return 2;
        }
        config.area_average = parsed != 0;
    }
    if (argc > 6 && !parse_int(argv[6], 1, 255, &config.threshold)) {
        (void)fprintf(stderr, "THRESHOLD must be an integer from 1 to 255\n");
        return 2;
    }
    if (argc > 7 &&
        !parse_int(argv[7], 1, INT_MAX, &config.contour_area)) {
        (void)fprintf(stderr, "CONTOUR_AREA must be a positive integer\n");
        return 2;
    }
    if (!kmd_detector_create(&detector, &config)) {
        (void)fprintf(stderr, "bad configuration\n");
        return 2;
    }
    frame_size = (size_t)config.width * (size_t)config.height;
    frame = malloc(frame_size);
    if (frame == NULL) {
        kmd_detector_free(detector);
        return 2;
    }

    for (;;) {
        const read_status status = read_full(frame, frame_size);

        if (status == READ_END) {
            break;
        }
        if (status == READ_TRUNCATED) {
            (void)fprintf(stderr, "truncated input frame\n");
            free(frame);
            kmd_detector_free(detector);
            return 1;
        }
        if (status == READ_ERROR) {
            (void)fprintf(stderr, "input read failed: %s\n", strerror(errno));
            free(frame);
            kmd_detector_free(detector);
            return 1;
        }
        {
            const size_t count =
                kmd_detect(detector, frame, boxes, 64u, &result);

            frames++;
            if (result.calibrating) {
                calibrating_frames++;
            }
            if (result.scene_changed) {
                scene_changes++;
            }
            /* Boxes during calibration are the startup burst, not findings. */
            if (!result.calibrating && result.found > 0u) {
                const uint64_t found = (uint64_t)result.found;

                motion_frames++;
                if (found > UINT64_MAX - total_boxes) {
                    total_boxes = UINT64_MAX;
                } else {
                    total_boxes += found;
                }
                if (printf("frame %" PRIu64 " boxes %zu fraction %.4f",
                           frames, result.found,
                           (double)result.motion_fraction) < 0) {
                    free(frame);
                    kmd_detector_free(detector);
                    return 1;
                }
                for (size_t i = 0u; i < count; i++) {
                    if (printf(" [%d,%d %dx%d]", boxes[i].x0, boxes[i].y0,
                               boxes[i].x1 - boxes[i].x0,
                               boxes[i].y1 - boxes[i].y0) < 0) {
                        free(frame);
                        kmd_detector_free(detector);
                        return 1;
                    }
                }
                if (printf("\n") < 0) {
                    free(frame);
                    kmd_detector_free(detector);
                    return 1;
                }
            }
        }
    }
    {
        const int summary = fprintf(
            stderr,
            "frames=%" PRIu64 " calibrating=%" PRIu64
            " motion_frames=%" PRIu64 " boxes=%" PRIu64
            " scene_changes=%" PRIu64 " motion_rate=%.1f%%\n",
            frames, calibrating_frames, motion_frames, total_boxes,
            scene_changes,
            frames > calibrating_frames
                ? 100.0 * (double)motion_frames /
                      (double)(frames - calibrating_frames)
                : 0.0);
        const int flush = fflush(stdout);

        free(frame);
        kmd_detector_free(detector);
        return summary < 0 || flush == EOF ? 1 : 0;
    }
}
