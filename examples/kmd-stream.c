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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static bool read_full(uint8_t *buffer, size_t size)
{
    size_t total = 0u;

    while (total < size) {
        const ssize_t got = read(STDIN_FILENO, buffer + total, size - total);

        if (got <= 0) {
            return false;
        }
        total += (size_t)got;
    }
    return true;
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
    long frames = 0;
    long motion_frames = 0;
    long calibrating_frames = 0;
    long scene_changes = 0;
    long total_boxes = 0;

    if (argc < 3) {
        (void)fprintf(stderr,
                      "usage: %s WIDTH HEIGHT [DETECT_HEIGHT] [CONTRAST 0|1] "
                      "[AREA_AVERAGE 0|1] [THRESHOLD] [CONTOUR_AREA]\n",
                      argv[0]);
        return 2;
    }
    kmd_config_init(&config);
    config.width = atoi(argv[1]);
    config.height = atoi(argv[2]);
    config.pixfmt = KMD_PIXFMT_GRAY;
    if (argc > 3) {
        config.detect_height = atoi(argv[3]);
    }
    if (argc > 4) {
        config.improve_contrast = atoi(argv[4]) != 0;
    }
    if (argc > 5) {
        config.area_average = atoi(argv[5]) != 0;
    }
    if (argc > 6) {
        config.threshold = atoi(argv[6]);
    }
    if (argc > 7) {
        config.contour_area = atoi(argv[7]);
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

    while (read_full(frame, frame_size)) {
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
            motion_frames++;
            total_boxes += (long)result.found;
            (void)printf("frame %ld boxes %zu fraction %.4f",
                         frames, result.found, (double)result.motion_fraction);
            for (size_t i = 0u; i < count; i++) {
                (void)printf(" [%d,%d %dx%d]", boxes[i].x0, boxes[i].y0,
                             boxes[i].x1 - boxes[i].x0,
                             boxes[i].y1 - boxes[i].y0);
            }
            (void)printf("\n");
        }
    }
    (void)fprintf(stderr,
                  "frames=%ld calibrating=%ld motion_frames=%ld "
                  "boxes=%ld scene_changes=%ld motion_rate=%.1f%%\n",
                  frames, calibrating_frames, motion_frames, total_boxes,
                  scene_changes,
                  frames > calibrating_frames
                      ? 100.0 * (double)motion_frames /
                            (double)(frames - calibrating_frames)
                      : 0.0);
    free(frame);
    kmd_detector_free(detector);
    return 0;
}
