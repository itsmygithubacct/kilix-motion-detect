#include "kilix_motion_detect.h"

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            (void)fprintf(stderr, "%s:%d: fuzz check failed: %s\n",          \
                          __FILE__, __LINE__, #condition);                    \
            return false;                                                     \
        }                                                                     \
    } while (false)

static uint64_t random_state = UINT64_C(0x4b4d4446555a5a01);

static uint32_t next_random(void)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 7;
    random_state ^= random_state << 17;
    return (uint32_t)(random_state >> 16);
}

static uint64_t hash_bytes(uint64_t hash, const void *value, size_t size)
{
    const uint8_t *bytes = value;

    for (size_t index = 0u; index < size; index++) {
        hash ^= bytes[index];
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

static bool reject_invalid(kmd_config config, unsigned choice)
{
    kmd_detector *detector = NULL;

    switch (choice % 12u) {
    case 0u: config.width = 0; break;
    case 1u: config.height = -1; break;
    case 2u: config.pixfmt = (kmd_pixfmt)99; break;
    case 3u: config.threshold = 0; break;
    case 4u: config.contour_area = 0; break;
    case 5u: config.frame_alpha = NAN; break;
    case 6u: config.lightning_threshold = INFINITY; break;
    case 7u: config.lightning_threshold = -0.1f; break;
    case 8u: config.skip_threshold = 1.1f; break;
    case 9u: config.detect_height = -1; break;
    case 10u: config.blur_radius = -1; break;
    default:
        config.width = INT_MAX;
        config.height = INT_MAX;
        break;
    }
    CHECK(!kmd_detector_create(&detector, &config));
    CHECK(detector == NULL);
    return true;
}

static bool run_case(uint64_t *checksum, size_t *frames)
{
    kmd_config config;
    kmd_detector *detector = NULL;
    kmd_box boxes[32];
    uint8_t *frame;
    uint8_t *mask;
    size_t pixels;
    size_t frame_size;
    size_t bytes_per_pixel;
    const bool use_mask = (next_random() & 1u) != 0u;

    kmd_config_init(&config);
    config.width = 1 + (int)(next_random() % 64u);
    config.height = 1 + (int)(next_random() % 48u);
    config.pixfmt = (kmd_pixfmt)(next_random() % 3u);
    config.detect_height =
        (int)(next_random() % (uint32_t)(config.height + 8));
    config.threshold = 1 + (int)(next_random() % 255u);
    config.contour_area = 1 + (int)(next_random() % 40u);
    config.frame_alpha =
        (float[]){0.001f, 0.01f, 0.2f, 1.0f}[next_random() % 4u];
    config.improve_contrast = (next_random() & 1u) != 0u;
    config.lightning_threshold =
        (float[]){0.0f, 0.3f, 0.8f, 1.0f}[next_random() % 4u];
    config.skip_threshold =
        (float[]){0.0f, 0.2f, 0.7f, 1.0f}[next_random() % 4u];
    config.area_average = (next_random() & 1u) != 0u;
    config.blur_radius =
        (next_random() & 31u) == 0u ? INT_MAX : (int)(next_random() % 10u);

    CHECK(reject_invalid(config, next_random()));
    CHECK(kmd_detector_create(&detector, &config));
    CHECK(detector != NULL);
    CHECK(kmd_detector_scaled_width(detector) >= 1);
    CHECK(kmd_detector_scaled_height(detector) >= 1);

    pixels = (size_t)config.width * (size_t)config.height;
    bytes_per_pixel = config.pixfmt == KMD_PIXFMT_GRAY ? 1u : 4u;
    frame_size = pixels * bytes_per_pixel;
    frame = malloc(frame_size);
    mask = malloc(pixels);
    CHECK(frame != NULL && mask != NULL);

    for (size_t index = 0u; index < frame_size; index++) {
        frame[index] = (uint8_t)next_random();
    }
    if (use_mask) {
        for (size_t index = 0u; index < pixels; index++) {
            mask[index] = (next_random() & 7u) == 0u ? 0u :
                          (uint8_t)(1u + next_random() % 255u);
        }
        CHECK(!kmd_detector_set_mask(
            detector, mask, pixels == 1u ? 0u : pixels - 1u));
        CHECK(kmd_detector_set_mask(detector, mask, pixels));
    }

    for (int frame_number = 0; frame_number < 10; frame_number++) {
        const size_t mutations =
            1u + (size_t)next_random() % (pixels / 4u + 1u);
        const size_t capacity =
            (size_t[]){0u, 1u, 8u, 32u}[next_random() % 4u];
        kmd_result result;
        size_t written;

        for (size_t mutation = 0u; mutation < mutations; mutation++) {
            const size_t pixel = (size_t)next_random() % pixels;

            for (size_t channel = 0u; channel < bytes_per_pixel; channel++) {
                frame[pixel * bytes_per_pixel + channel] =
                    (uint8_t)next_random();
            }
        }
        written = kmd_detect(
            detector, frame, capacity == 0u ? NULL : boxes, capacity, &result);
        CHECK(written == result.count);
        CHECK(written <= capacity);
        CHECK(result.count <= result.found);
        CHECK(isfinite(result.motion_fraction));
        CHECK(result.motion_fraction >= 0.0f && result.motion_fraction <= 1.0f);
        CHECK(!result.skipped || (result.count == 0u && result.found == 0u));
        for (size_t index = 0u; index < written; index++) {
            CHECK(boxes[index].x0 >= 0 && boxes[index].y0 >= 0);
            CHECK(boxes[index].x1 > boxes[index].x0);
            CHECK(boxes[index].y1 > boxes[index].y0);
            CHECK(boxes[index].x1 <= config.width);
            CHECK(boxes[index].y1 <= config.height);
        }
        *checksum = hash_bytes(*checksum, &result.count, sizeof(result.count));
        *checksum = hash_bytes(*checksum, &result.found, sizeof(result.found));
        *checksum = hash_bytes(
            *checksum, &result.motion_fraction, sizeof(result.motion_fraction));
        *checksum = hash_bytes(*checksum, boxes, written * sizeof(boxes[0]));
        (*frames)++;
        if (frame_number == 4 && (next_random() & 3u) == 0u) {
            kmd_detector_reset(detector);
        }
    }
    CHECK(kmd_detector_set_mask(detector, NULL, 0u));
    free(mask);
    free(frame);
    kmd_detector_free(detector);
    return true;
}

int main(int argc, char **argv)
{
    size_t cases = 10000u;
    size_t frames = 0u;
    uint64_t checksum = UINT64_C(1469598103934665603);

    if (argc == 2) {
        char *end = NULL;
        uintmax_t parsed;

        errno = 0;
        parsed = strtoumax(argv[1], &end, 10);
        if (errno != 0 || end == argv[1] || *end != '\0' || parsed == 0u ||
            parsed > SIZE_MAX / 10u) {
            return 2;
        }
        cases = (size_t)parsed;
    } else if (argc != 1) {
        return 2;
    }
    for (size_t index = 0u; index < cases; index++) {
        if (!run_case(&checksum, &frames)) {
            (void)fprintf(stderr, "fuzz case %zu failed\n", index);
            return 1;
        }
    }
    (void)printf("fuzz cases=%zu frames=%zu checksum=%016" PRIx64 "\n",
                 cases, frames, checksum);
    return 0;
}
