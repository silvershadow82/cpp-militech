#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdarg.h>

#include <freertos/FreeRTOS.h>   /* portMUX_TYPE, portENTER_CRITICAL */
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h> 
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/uart.h>

#include <i2cdev.h>
#include <pcf8574.h>
#include <hd44780.h>
#include <buzzer.h>
#include "zones.h"

static const char *TAG = "fara";

static constexpr uint8_t    LCD_ADDR          = 0x3f;
static constexpr i2c_port_t I2C_PORT          = I2C_NUM_0;
static constexpr gpio_num_t I2C_SDA_GPIO      = GPIO_NUM_21;
static constexpr gpio_num_t I2C_SCL_GPIO      = GPIO_NUM_22;

static constexpr int      BUZZER_GPIO    = 4;
static constexpr uint32_t BUZZER_FREQ_HZ = 2700;

static constexpr gpio_num_t TRIGGER_GPIO    = GPIO_NUM_32;
static constexpr gpio_num_t ECHO_GPIO       = GPIO_NUM_33;
static constexpr uint32_t   MAX_DISTANCE_CM = 150;

static constexpr uint32_t TICK_MS      = 10;
static constexpr uint32_t TIMER_RES_HZ  = 1000000;
static constexpr uint64_t TIMER_ALARM   = TICK_MS * (TIMER_RES_HZ / 1000);

static constexpr uint32_t SENSOR_PERIOD_MS  = 100;
static constexpr uint32_t DISPLAY_PERIOD_MS = 250;

static gptimer_handle_t s_tick_timer;

static volatile uint32_t s_tick_count;

extern "C" bool IRAM_ATTR fara_on_tick(gptimer_handle_t,
                                       const gptimer_alarm_event_data_t *,
                                       void *)
{
    s_tick_count = s_tick_count + 1;
    return false;
}

static esp_err_t tick_timer_start()
{
    gptimer_config_t cfg = {};
    cfg.clk_src       = GPTIMER_CLK_SRC_DEFAULT;
    cfg.direction     = GPTIMER_COUNT_UP;
    cfg.resolution_hz = TIMER_RES_HZ;

    esp_err_t err = gptimer_new_timer(&cfg, &s_tick_timer);
    if (err != ESP_OK) {
        return err;
    }

    gptimer_event_callbacks_t cbs = { .on_alarm = fara_on_tick };
    err = gptimer_register_event_callbacks(s_tick_timer, &cbs, NULL);
    if (err != ESP_OK) {
        return err;
    }

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count  = TIMER_ALARM;
    alarm.reload_count = 0;
    alarm.flags.auto_reload_on_alarm = true;
    err = gptimer_set_alarm_action(s_tick_timer, &alarm);
    if (err != ESP_OK) {
        return err;
    }

    err = gptimer_enable(s_tick_timer);
    if (err != ESP_OK) {
        return err;
    }
    return gptimer_start(s_tick_timer);
}


static constexpr uint32_t ECHO_TIMEOUT_MS = 40;   /* longer than echo */
static constexpr uint32_t ECHO_MIN_US     = 100;  /* noise */
static constexpr uint32_t US_PER_CM       = 58;   /* round trip of 1 cm at the speed of sound */

static portMUX_TYPE s_echo_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile int64_t  s_echo_rise_us;
static volatile uint32_t s_echo_width_us;
static volatile bool     s_echo_ready;

extern "C" void IRAM_ATTR fara_on_echo_edge(void *)
{
    int64_t now = esp_timer_get_time();

    if (gpio_get_level(ECHO_GPIO)) {
        s_echo_rise_us = now;
        return;
    }

    if (s_echo_rise_us == 0) {
        return;
    }

    portENTER_CRITICAL_ISR(&s_echo_mux);
    s_echo_width_us = (uint32_t)(now - s_echo_rise_us);
    s_echo_ready    = true;
    portEXIT_CRITICAL_ISR(&s_echo_mux);

    s_echo_rise_us = 0;
}

// Init ultrasonic sensor
static esp_err_t sensor_init()
{
    gpio_config_t trig = {};
    trig.pin_bit_mask = 1ULL << TRIGGER_GPIO;
    trig.mode         = GPIO_MODE_OUTPUT;
    trig.intr_type    = GPIO_INTR_DISABLE;
    esp_err_t err = gpio_config(&trig);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(TRIGGER_GPIO, 0);

    gpio_config_t echo = {};
    echo.pin_bit_mask = 1ULL << ECHO_GPIO;
    echo.mode         = GPIO_MODE_INPUT;
    echo.intr_type    = GPIO_INTR_ANYEDGE;
    err = gpio_config(&echo);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_isr_handler_add(ECHO_GPIO, fara_on_echo_edge, NULL);
}

static void ping_start()
{
    portENTER_CRITICAL(&s_echo_mux);
    s_echo_ready   = false;
    s_echo_rise_us = 0;
    portEXIT_CRITICAL(&s_echo_mux);

    gpio_set_level(TRIGGER_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIGGER_GPIO, 0);
}

static constexpr size_t   SENSOR_WINDOW      = 5;
static constexpr uint32_t SENSOR_ERROR_LIMIT = 5;
static constexpr uint32_t SENSOR_MISS_LIMIT = 3;   // 300 ms at 10 Hz

typedef struct {
    uint32_t    distance_cm;
    fara_zone_t zone;
    bool        in_range;
    uint32_t    error_count;
} fara_state_t;

static fara_state_t s_state = {
    .distance_cm = 0,
    .zone        = FARA_ZONE_CLEAR,
    .in_range    = false,
    .error_count = 0,
};

static uint32_t s_window[SENSOR_WINDOW];
static size_t   s_filled;
static size_t   s_next_sample;

static bool     s_ping_pending;
static uint32_t s_ping_deadline_ms;
static uint32_t s_miss_streak;

/* One ping came back with nothing usable. */
static void sensor_miss()
{
    if (s_miss_streak < SENSOR_MISS_LIMIT) {
        s_miss_streak++;
    }
 
    s_state.error_count = 0;

    if (s_miss_streak < SENSOR_MISS_LIMIT) {
        return;
    }

    s_filled      = 0;
    s_next_sample = 0;
    s_state.in_range = false;
    s_state.zone     = FARA_ZONE_CLEAR;
}

static void sensor_accept(uint32_t cm)
{
    s_window[s_next_sample] = cm;
    s_next_sample = (s_next_sample + 1) % SENSOR_WINDOW;
    if (s_filled < SENSOR_WINDOW) {
        s_filled++;
    }

    s_state.distance_cm = fara_median5(s_window, s_filled);
    s_state.zone        = fara_zone(s_state.distance_cm, s_state.zone);
    s_state.in_range    = true;
    s_state.error_count = 0;
    s_miss_streak       = 0;
}

static void sensor_step(uint32_t now_ms)
{
    bool     ready;
    uint32_t width_us;

    portENTER_CRITICAL(&s_echo_mux);
    ready    = s_echo_ready;
    width_us = s_echo_width_us;
    s_echo_ready = false;
    portEXIT_CRITICAL(&s_echo_mux);

    if (!s_ping_pending) {
        return;
    }

    if (ready) {
        s_ping_pending = false;

        if (width_us < ECHO_MIN_US) {
            // Jitters
            sensor_miss();
            return;
        }

        uint32_t cm = width_us / US_PER_CM;
        if (cm > MAX_DISTANCE_CM) {
            sensor_miss();
        } else {
            sensor_accept(cm);
        }
        return;
    }

    if ((int32_t)(now_ms - s_ping_deadline_ms) >= 0) {
        s_ping_pending = false;
        sensor_miss();
    }
}

static void sensor_ping(uint32_t now_ms)
{
    if (s_ping_pending || gpio_get_level(ECHO_GPIO)) {
        if (s_state.error_count < SENSOR_ERROR_LIMIT) {
            s_state.error_count++;
        }
        return;
    }

    ping_start();
    s_ping_pending     = true;
    s_ping_deadline_ms = now_ms + ECHO_TIMEOUT_MS;
}

typedef struct {
    uint32_t on_ms;   /* 0 = silent. */
    uint32_t off_ms;  /* 0 with a non-zero on_ms = continuous. */
} cadence_t;

static const cadence_t k_cadence[4] = {
    { 80,   0 },   /* BADABOOM - continuous */
    { 80, 220 },   /* NEAR */
    { 80, 720 },   /* FAR */
    {  0,   0 },   /* CLEAR - silent */
};

static bool        s_buzzer_on;
static fara_zone_t s_buzzer_zone = FARA_ZONE_CLEAR;
static uint32_t    s_buzzer_since_ms;

static void buzzer_set(bool on)
{
    if (on == s_buzzer_on) {
        return;
    }
    s_buzzer_on = on;
    (void)(on ? buzzer_on() : buzzer_off());
}

static fara_zone_t active_zone()
{
    if (s_state.error_count >= SENSOR_ERROR_LIMIT) {
        return FARA_ZONE_CLEAR;
    }
    return s_state.zone;
}

static void buzzer_step(uint32_t now_ms)
{
    fara_zone_t zone = active_zone();

    if (zone != s_buzzer_zone) {
        s_buzzer_zone     = zone;
        s_buzzer_since_ms = now_ms;
    }

    cadence_t c = k_cadence[zone];

    if (c.on_ms == 0) {
        buzzer_set(false);
        return;
    }
    if (c.off_ms == 0) {
        buzzer_set(true);
        return;
    }

    uint32_t phase = (now_ms - s_buzzer_since_ms) % (c.on_ms + c.off_ms);
    buzzer_set(phase < c.on_ms);
}

/* ------------------------------------------------------------------ */
/* LCD                                                                */
/* ------------------------------------------------------------------ */

static constexpr size_t LCD_COLS = 16;

static i2c_dev_t s_pcf8574;
static bool      s_lcd_ready;

extern "C" esp_err_t fara_lcd_write(const hd44780_t *, uint8_t data)
{
    return pcf8574_port_write(&s_pcf8574, data);
}

static hd44780_t s_lcd = {
    .write_cb = fara_lcd_write,
    .pins = {
        .rs = 0,
        .e  = 2,
        .d4 = 4,
        .d5 = 5,
        .d6 = 6,
        .d7 = 7,
        .bl = 3,
    },
    .font      = HD44780_FONT_5X8,
    .lines     = 2,
    .backlight = false,
};

static void render_line(char *dst, const char *text)
{
    size_t i = 0;
    while (text[i] != '\0' && i < LCD_COLS) {
        dst[i] = text[i];
        i++;
    }
    while (i < LCD_COLS) {
        dst[i] = ' ';
        i++;
    }
    dst[LCD_COLS] = '\0';
}

static char s_shown[2][LCD_COLS + 1];

static void display_step()
{
    char next[2][LCD_COLS + 1];
    char scratch[32];

    if (s_state.error_count >= SENSOR_ERROR_LIMIT) {
        render_line(next[0], "Distance:  ---");
        render_line(next[1], "SENSOR ERR");
    } else {
        if (s_state.in_range) {
            snprintf(scratch, sizeof(scratch), "Distance: %3" PRIu32 "cm",
                     s_state.distance_cm);
        } else {
            snprintf(scratch, sizeof(scratch), "Distance:  ---");
        }
        render_line(next[0], scratch);

        snprintf(scratch, sizeof(scratch), "Zone: %s",
                 fara_zone_label(s_state.zone));
        render_line(next[1], scratch);
    }

    for (int line = 0; line < 2; line++) {
        if (strcmp(s_shown[line], next[line]) == 0) {
            continue;
        }

        esp_err_t goto_err = hd44780_gotoxy(&s_lcd, 0, line);
        esp_err_t puts_err = hd44780_puts(&s_lcd, next[line]);

        if (goto_err == ESP_OK && puts_err == ESP_OK) {
            strcpy(s_shown[line], next[line]);
        }
    }
}

static void lcd_start()
{
    ESP_ERROR_CHECK(i2cdev_init());

    memset(&s_pcf8574, 0, sizeof(s_pcf8574));
    esp_err_t err = pcf8574_init_desc(&s_pcf8574, LCD_ADDR, I2C_PORT,
                                      I2C_SDA_GPIO, I2C_SCL_GPIO);
    if (err == ESP_OK) {
        err = hd44780_init(&s_lcd);
    }

    if (err == ESP_OK) {
        hd44780_switch_backlight(&s_lcd, true);
        s_lcd_ready = true;
    } else {
        ESP_LOGE(TAG, "LCD init failed at 0x%02x (%s)", LCD_ADDR,
                 esp_err_to_name(err));
    }
}

static constexpr uart_port_t CONSOLE_UART     = UART_NUM_2;
static constexpr gpio_num_t  CONSOLE_RX_GPIO  = GPIO_NUM_25;
static constexpr gpio_num_t  CONSOLE_TX_GPIO  = GPIO_NUM_26;
static constexpr int         CONSOLE_BAUD     = 115200;
static constexpr int         CONSOLE_RX_BUF   = 256;
static constexpr int         CONSOLE_TX_BUF   = 256;
static constexpr size_t      CONSOLE_LINE_MAX = 32;

static bool   s_console_ready;
static char   s_line[CONSOLE_LINE_MAX];
static size_t s_line_len;

static void console_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void console_printf(const char *fmt, ...)
{
    if (!s_console_ready) {
        return;
    }

    char    out[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out, sizeof(out), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    size_t len = (size_t)n < sizeof(out) ? (size_t)n : sizeof(out) - 1;
    uart_write_bytes(CONSOLE_UART, out, len);
}

static void console_usage()
{
    console_printf("badaboom = %" PRIu32 " cm; send a number %d-%d to change it\r\n",
                   fara_badaboom_cm(), FARA_BADABOOM_MIN_CM, FARA_BADABOOM_MAX_CM);

    ESP_LOGI(TAG, "badaboom = %" PRIu32 " cm; send a number %d-%d to change it",
             fara_badaboom_cm(), FARA_BADABOOM_MIN_CM, FARA_BADABOOM_MAX_CM);
}

static void console_line(const char *line)
{
    while (*line == ' ' || *line == '\t') {
        line++;
    }

    if (*line < '0' || *line > '9') {
        console_usage();
        return;
    }

    uint32_t cm = 0;
    while (*line >= '0' && *line <= '9' && cm <= 10000) {
        cm = cm * 10 + (uint32_t)(*line - '0');
        line++;
    }

    if (!fara_set_badaboom_cm(cm)) {
        console_printf("rejected %" PRIu32 " cm: allowed range is %d-%d\r\n",
                       cm, FARA_BADABOOM_MIN_CM, FARA_BADABOOM_MAX_CM);
        ESP_LOGW(TAG, "rejected %" PRIu32 " cm: allowed range is %d-%d",
                 cm, FARA_BADABOOM_MIN_CM, FARA_BADABOOM_MAX_CM);
        return;
    }

    console_printf("badaboom distance set to %" PRIu32 " cm\r\n", cm);
    
    ESP_LOGI(TAG, "badaboom distance set to %" PRIu32 " cm over the console", cm);
}

static void console_init()
{
    uart_config_t cfg = {};
    cfg.baud_rate  = CONSOLE_BAUD;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(CONSOLE_UART, &cfg);
    if (err == ESP_OK) {
        err = uart_set_pin(CONSOLE_UART, CONSOLE_TX_GPIO, CONSOLE_RX_GPIO,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        err = uart_driver_install(CONSOLE_UART, CONSOLE_RX_BUF, CONSOLE_TX_BUF,
                                  0, NULL, 0);
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "console unavailable (%s) - distance stays fixed",
                 esp_err_to_name(err));
        return;
    }

    s_console_ready = true;
    ESP_LOGI(TAG, "console on UART%d, rx=%d tx=%d @ %d baud",
             (int)CONSOLE_UART, (int)CONSOLE_RX_GPIO, (int)CONSOLE_TX_GPIO,
             CONSOLE_BAUD);
    console_printf("\r\nfara console\r\n");
    console_usage();
}

static void console_step()
{
    if (!s_console_ready) {
        return;
    }

    uint8_t buf[64];
    int n = uart_read_bytes(CONSOLE_UART, buf, sizeof(buf), 0);

    for (int i = 0; i < n; i++) {
        char ch = (char)buf[i];

        if (ch == '\n' || ch == '\r') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                console_line(s_line);
                s_line_len = 0;
            }
            continue;
        }

        if (s_line_len + 1 < sizeof(s_line)) {
            s_line[s_line_len++] = ch;
        } else {
            s_line_len = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* The loop                                                            */
/* ------------------------------------------------------------------ */

extern "C" void app_main()
{
    ESP_ERROR_CHECK(buzzer_init(BUZZER_GPIO, BUZZER_FREQ_HZ));
    ESP_ERROR_CHECK(sensor_init());
    ESP_ERROR_CHECK(tick_timer_start());

    console_init();
    lcd_start();

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "fara started");

    uint32_t ticks_done   = s_tick_count;
    uint32_t overruns     = 0;
    uint32_t next_ping_ms = 0;
    uint32_t next_lcd_ms  = 0;
    uint32_t next_log_ms  = 0;

    while(1) {
        uint32_t ticks   = s_tick_count;
        uint32_t pending = ticks - ticks_done;

        if (pending == 0) {
            continue;
        }

        if (pending > 1) {
            // got new tick while previous is running
            overruns += pending - 1;
        }
        ticks_done = ticks;

        uint32_t now_ms = ticks * TICK_MS;

        console_step();

        if ((int32_t)(now_ms - next_ping_ms) >= 0) {
            next_ping_ms = now_ms + SENSOR_PERIOD_MS;
            sensor_ping(now_ms);
        }

        sensor_step(now_ms);
        buzzer_step(now_ms);

        if (s_lcd_ready && (int32_t)(now_ms - next_lcd_ms) >= 0) {
            next_lcd_ms = now_ms + DISPLAY_PERIOD_MS;
            display_step();
        }

        if ((int32_t)(now_ms - next_log_ms) >= 0) {
            next_log_ms = now_ms + SENSOR_PERIOD_MS;

            if (s_state.in_range) {
                printf("t=%" PRIu32 " ms distance = %" PRIu32 " cm, zone = %s, overruns = %" PRIu32 "\n",
                       now_ms, s_state.distance_cm, fara_zone_label(s_state.zone), overruns);
            } else {
                printf("t=%" PRIu32 " ms distance = ---, zone = %s, overruns = %" PRIu32 "\n",
                       now_ms, fara_zone_label(s_state.zone), overruns);
            }
        }

        esp_task_wdt_reset();
    }
}
