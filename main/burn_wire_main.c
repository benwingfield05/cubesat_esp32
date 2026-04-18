/* burn_wire_console_trigger.c

   ESP-IDF example:
   - waits for a console activation key
   - after valid input, waits 30 seconds
   - activates GPIO_FIRE_EN for 5 seconds
   - monitors a limit switch and reports:
       * PANELS_CLOSED   when switch is pressed
       * PANELS_DEPLOYED when switch is not pressed

   Use with idf.py monitor and type:
       y
   then press Enter.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "burn_wire";

/* Onboard LED GPIO comes from the blink example's config */
#define LED_GPIO CONFIG_BLINK_GPIO

/* Burn wire output */
#define GPIO_FIRE_EN GPIO_NUM_7

/* Limit switch input */
#define LIMIT_SWITCH_GPIO GPIO_NUM_0

/* Assuming switch is wired to GND with pull-up enabled:
   pressed   -> 0
   released  -> 1
   If your wiring is reversed, change this to 1.
*/
#define LIMIT_SWITCH_PRESSED_LEVEL 1

/* Console activation settings */
#define ACTIVATION_KEY     'y'
#define CANCEL_KEY         'c'
#define ARM_DELAY_SEC      1800
#define FIRE_DURATION_SEC  10

/* Polling / debounce */
#define LIMIT_SWITCH_POLL_MS  30

/* LED state */
static uint8_t status_led_state = 0;

/* Prevent overlapping fire sequences */
static volatile bool fire_sequence_running = false;

/* Enable cancellation of arming the burn wire */
static volatile bool cancel_requested = false;

#ifdef CONFIG_BLINK_LED_STRIP
static led_strip_handle_t led_strip;

static void led_apply_state(void)
{
    if (status_led_state) {
        /* Green while FIRE_EN is active */
        led_strip_set_pixel(led_strip, 0, 0, 32, 0);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configured to use addressable onboard LED (LED strip).");

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
    };

#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif

    status_led_state = 0;
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void led_apply_state(void)
{
    gpio_set_level(LED_GPIO, status_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configured to use GPIO LED.");
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
}

#else
#error "unsupported LED type"
#endif

static void set_status_led(bool on)
{
    status_led_state = on ? 1 : 0;
    led_apply_state();
}

static void configure_wire(void)
{
    gpio_reset_pin(GPIO_FIRE_EN);
    gpio_set_direction(GPIO_FIRE_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_FIRE_EN, 0);
}

static void configure_limit_switch(void)
{
    gpio_config_t limit_switch_config = {
        .pin_bit_mask = (1ULL << LIMIT_SWITCH_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&limit_switch_config));
}

static inline bool limit_switch_pressed(int level)
{
    return (level == LIMIT_SWITCH_PRESSED_LEVEL);
}

static void report_panel_state(bool pressed)
{
    if (pressed) {
        ESP_LOGI(TAG, "STATUS: PANELS_CLOSED");
    } else {
        ESP_LOGI(TAG, "STATUS: PANELS_DEPLOYED");
    }
}

static bool is_activation_key(char c)
{
    return (c == ACTIVATION_KEY);
}

static bool is_cancel_key(char c)
{
    return (c == CANCEL_KEY);
}

static bool cancellation_requested(void)
{
    return cancel_requested;
}

static void activate_burn_wire_sequence(void)
{
    fire_sequence_running = true;

    ESP_LOGW(TAG, "Activation accepted. Waiting %d seconds before firing...", ARM_DELAY_SEC);

    for (int i = ARM_DELAY_SEC; i > 0; --i) {
        if (cancellation_requested()) {
            ESP_LOGW(TAG, "Sequence cancelled during arming delay.");
            gpio_set_level(GPIO_FIRE_EN, 0);
            fire_sequence_running = false;
            cancel_requested = false;
            return;
        }

        if ((i % 60 == 0) || (i <= 10)) {
            ESP_LOGI(TAG, "T-minus %d minute%s, %d second%s",
                     i / 60, ((i / 60) == 1) ? "" : "s",
                     i % 60, ((i % 60) == 1) ? "" : "s");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (cancellation_requested()) {
            ESP_LOGW(TAG, "Sequence cancelled during arming delay.");
            gpio_set_level(GPIO_FIRE_EN, 0);
            fire_sequence_running = false;
            cancel_requested = false;
            return;
    }

    ESP_LOGW(TAG, "Activating GPIO_FIRE_EN for %d seconds.", FIRE_DURATION_SEC);
    gpio_set_level(GPIO_FIRE_EN, 1);
    set_status_led(true);

    vTaskDelay(pdMS_TO_TICKS(FIRE_DURATION_SEC * 1000));

    gpio_set_level(GPIO_FIRE_EN, 0);
    set_status_led(false);

    ESP_LOGI(TAG, "Burn complete. GPIO_FIRE_EN deactivated.");

    fire_sequence_running = false;
    cancel_requested = false;
}

static void console_task(void *pvParameters)
{
    char line[32];
    bool prompt_shown = false;

    while (1) {
        if (!prompt_shown) {
            ESP_LOGI(TAG, "Type '%c' + Enter to arm, '%c' + Enter to cancel.",
                     ACTIVATION_KEY, CANCEL_KEY);
            prompt_shown = true;
        }

        if (fgets(line, sizeof(line), stdin) == NULL) {
            /* Do not spam the prompt if stdin is not ready */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Input arrived, so the next loop may show the prompt again */
        prompt_shown = false;

        if (fire_sequence_running) {
            ESP_LOGW(TAG, "Fire sequence already in progress; input ignored.");
            continue;
        }
        
        // If the activation key is given, go with the burnwire sequence
        if (is_activation_key(line[0])) {
            activate_burn_wire_sequence();
        } else if (is_cancel_key(line[0])) {
            if (fire_sequence_running) {
                cancel_requested = true;
                ESP_LOGW(TAG, "Manual cancel requested.");
            } else {
                ESP_LOGI(TAG, "No active fire sequence to cancel.");
            }
        } else {
            if (line[0] == '\n' || line[0] == '\0') {
                ESP_LOGI(TAG, "No activation key entered.");
            } else {
                ESP_LOGI(TAG, "Ignored input: %c", line[0]);
            }
        }
    }
}

static void limit_switch_task(void *pvParameters)
{
    int level = gpio_get_level(LIMIT_SWITCH_GPIO);
    bool pressed = limit_switch_pressed(level);
    bool prev_pressed = pressed;

    /* Report initial state once */
    report_panel_state(pressed);

    while (1) {
        level = gpio_get_level(LIMIT_SWITCH_GPIO);
        pressed = limit_switch_pressed(level);

        if (pressed != prev_pressed) {
            prev_pressed = pressed;
            report_panel_state(pressed);
        }

        vTaskDelay(pdMS_TO_TICKS(LIMIT_SWITCH_POLL_MS));
    }
}

void app_main(void)
{
    configure_led();
    configure_wire();
    configure_limit_switch();
    set_status_led(false);

    ESP_LOGI(TAG, "Started. LED_GPIO=%d, FIRE_EN=%d, LIMIT_SWITCH_GPIO=%d",
             (int)LED_GPIO, (int)GPIO_FIRE_EN, (int)LIMIT_SWITCH_GPIO);

    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
    xTaskCreate(limit_switch_task, "limit_switch_task", 2048, NULL, 5, NULL);
}