#include "zones.h"

/* Upper bound of each zone. k_bounds[0] is settable at runtime over the
 * console; the other two are fixed. Only the main loop touches it. */
static uint32_t k_bounds[FARA_MAX_BOUNDS] = { 10, 50, 100 };

uint32_t fara_badaboom_cm(void)
{
    return k_bounds[0];
}

bool fara_set_badaboom_cm(uint32_t cm)
{
    if (cm < FARA_BADABOOM_MIN_CM || cm > FARA_BADABOOM_MAX_CM) {
        return false;
    }
    k_bounds[0] = cm;
    return true;
}


fara_zone_t fara_zone(uint32_t cm, fara_zone_t prev)
{
    for (int i = 0; i < FARA_MAX_BOUNDS; i++) {
        uint32_t bound = k_bounds[i];

        if (prev <= (fara_zone_t)i) {
            bound += FARA_JITTER_CM;
        } else {
            bound -= FARA_JITTER_CM;
        }

        if (cm < bound) {
            return (fara_zone_t)i;
        }
    }

    return FARA_ZONE_CLEAR;
}

const char *fara_zone_label(fara_zone_t zone)
{
    switch (zone) {
    case FARA_ZONE_BADABOOM:  return "BADABOOM";
    case FARA_ZONE_NEAR:  return "NEAR";
    case FARA_ZONE_FAR:   return "FAR";
    case FARA_ZONE_CLEAR: return "CLEAR";
    default:              return "?";
    }
}

uint32_t fara_median5(const uint32_t *samples, size_t count)
{
    if (count == 0) {
        return 0;
    }

    uint32_t sorted[FARA_MAX_SAMPLES];
    if (count > FARA_MAX_SAMPLES) {
        count = FARA_MAX_SAMPLES;
    }
    for (size_t i = 0; i < count; i++) {
        sorted[i] = samples[i];
    }

    for (size_t i = 1; i < count; i++) {
        uint32_t current = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j - 1] > current) {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = current;
    }

    return sorted[count / 2];
}
