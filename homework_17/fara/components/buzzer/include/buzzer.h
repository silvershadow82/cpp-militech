#pragma once

#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t buzzer_init(int gpio_num, uint32_t freq_hz);
esp_err_t buzzer_on();
esp_err_t buzzer_off();

#ifdef __cplusplus
}
#endif
