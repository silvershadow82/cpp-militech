#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Ordered by increasing distance. fara_zone relies on this ordering. */
typedef enum {
    FARA_ZONE_BADABOOM  = 0,
    FARA_ZONE_NEAR      = 1,
    FARA_ZONE_FAR       = 2,
    FARA_ZONE_CLEAR     = 3,
} fara_zone_t;

#define FARA_MAX_BOUNDS  3
#define FARA_MAX_SAMPLES 5
#define FARA_JITTER_CM   3
#define FARA_BADABOOM_MIN_CM 5
#define FARA_BADABOOM_MAX_CM 45


fara_zone_t fara_zone(uint32_t cm, fara_zone_t prev);

/** The BADABOOM threshold currently in force, in cm. */
uint32_t fara_badaboom_cm(void);

bool fara_set_badaboom_cm(uint32_t cm);

/** Short label for the LCD: "BADABOOM", "NEAR", "FAR", "CLEAR". */
const char *fara_zone_label(fara_zone_t zone);

/** Median of up to 5 samples. Returns 0 if count is 0. */
uint32_t fara_median5(const uint32_t *samples, size_t count);

#ifdef __cplusplus
}
#endif
