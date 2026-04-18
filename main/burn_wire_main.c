/* burn_wire_console_trigger.c

   ESP-IDF example: wait for a console key, delay 30 seconds,
   then activate GPIO_FIRE_EN for 5 seconds.

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

/* Output pin for the burn wire activation */
#define GPIO_FIRE_EN GPIO_NUM_7

/* Console activation settings */
#define ACTIVATION_KEY   'y'
#define ARM_DELAY_SEC    30
#define FIRE_DURATION_SEC 5

/* LED state */
static uint8_t status_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP
static led_strip_handle_t led_strip;

static void led_apply_state(void)
{
    if (status_led_state) {
        /* Green while active */
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

static void configure_wire(void)
{
    gpio_reset_pin(GPIO_FIRE_EN);
    gpio_set_direction(GPIO_FIRE_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_FIRE_EN, 0);
}

static void set_status_led(bool on)
{
    status_led_state = on ? 1 : 0;
    led_apply_state();
}

static bool is_activation_key(char c)
{
    return (c == ACTIVATION_KEY || c == 'Y');
}

static bool wait_for_console_trigger(void)
{
    char line[32];

    ESP_LOGI(TAG, "Type '%c' then press Enter to begin activation.", ACTIVATION_KEY);

    if (fgets(line, sizeof(line), stdin) == NULL) {
        ESP_LOGW(TAG, "Console read failed.");
        return false;
    }

    if (is_activation_key(line[0])) {
        return true;
    }

    ESP_LOGI(TAG, "Ignored input: %c", line[0] ? line[0] : ' ');
    return false;
}

static void activate_burn_wire_sequence(void)
{
    ESP_LOGW(TAG, "Activation accepted. Waiting %d seconds before firing...", ARM_DELAY_SEC);

    for (int i = ARM_DELAY_SEC; i > 0; --i) {
        ESP_LOGI(TAG, "T-minus %d second%s", i, (i == 1) ? "" : "s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGW(TAG, "Activating GPIO_FIRE_EN for %d seconds.", FIRE_DURATION_SEC);
    gpio_set_level(GPIO_FIRE_EN, 1);
    set_status_led(true);

    vTaskDelay(pdMS_TO_TICKS(FIRE_DURATION_SEC * 1000));

    gpio_set_level(GPIO_FIRE_EN, 0);
    set_status_led(false);

    ESP_LOGI(TAG, "Burn complete. GPIO_FIRE_EN deactivated.");
}

void app_main(void)
{
    configure_led();
    configure_wire();
    set_status_led(false);

    ESP_LOGI(TAG, "Started. LED_GPIO=%d, FIRE_EN=%d",
             (int)LED_GPIO, (int)GPIO_FIRE_EN);

    while (1) {
        if (wait_for_console_trigger()) {
            activate_burn_wire_sequence();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}