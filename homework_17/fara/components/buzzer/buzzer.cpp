#include "buzzer.h"

#include <driver/ledc.h>

#define BUZZER_MODE       LEDC_LOW_SPEED_MODE
#define BUZZER_TIMER      LEDC_TIMER_0
#define BUZZER_CHANNEL    LEDC_CHANNEL_0
#define BUZZER_RESOLUTION LEDC_TIMER_10_BIT

/*
 * Volume knob, for an ACTIVE buzzer whose loudness follows the average
 * voltage its own oscillator is fed. Of a 10-bit range:
 *
 *   512 (50%, ~1.65 V) - silent: below the oscillator's start voltage
 *   700 (68%, ~2.25 V) - audible but subdued
 *   921 (90%, ~2.97 V) - full volume
 *
 * The floor is a hard edge rather than a fade, so if a lower value goes
 * silent it is under the start voltage, not merely quiet. Duty cannot reach
 * 1024: a constant level has no edges, and Wokwi's passive piezo needs them.
 */
#define BUZZER_DUTY_ON  700
#define BUZZER_DUTY_OFF 0

esp_err_t buzzer_init(int gpio_num, uint32_t freq_hz)
{
    ledc_timer_config_t timer = {};
    timer.speed_mode      = BUZZER_MODE;
    timer.duty_resolution = BUZZER_RESOLUTION;
    timer.timer_num       = BUZZER_TIMER;
    timer.freq_hz         = freq_hz;
    timer.clk_cfg         = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel = {};
    channel.gpio_num   = gpio_num;
    channel.speed_mode = BUZZER_MODE;
    channel.channel    = BUZZER_CHANNEL;
    channel.timer_sel  = BUZZER_TIMER;
    channel.duty       = BUZZER_DUTY_OFF;
    channel.hpoint     = 0;

    return ledc_channel_config(&channel);
}

static esp_err_t buzzer_set_duty(uint32_t duty)
{
    esp_err_t err = ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
}

esp_err_t buzzer_on()
{
    return buzzer_set_duty(BUZZER_DUTY_ON);
}

esp_err_t buzzer_off()
{
    return buzzer_set_duty(BUZZER_DUTY_OFF);
}
