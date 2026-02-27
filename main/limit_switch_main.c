/* limit_switch_led_main.c

   ESP-IDF example: read a limit switch on an ESP32-C6 GPIO and control the
   onboard LED (either a simple GPIO LED or an addressable LED strip LED),
   following the same LED setup style as blink_example_main.c.

   Configuration (idf.py menuconfig):
     - Set the onboard LED pin via CONFIG_BLINK_GPIO (same as blink example)
     - Choose LED type: CONFIG_BLINK_LED_GPIO or CONFIG_BLINK_LED_STRIP (+ backend)
     - Add a Kconfig entry for the limit switch GPIO:
         CONFIG_LIMIT_SWITCH_GPIO = <GPIO number>
       If you don't have Kconfig set up, you can replace CONFIG_LIMIT_SWITCH_GPIO
       with a literal GPIO number below.
*/

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "burn_wire:";

/* User-configurable GPIOs */

/* Onboard LED GPIO comes from the blink example's config */
#define LED_GPIO CONFIG_BLINK_GPIO

/* Limit switch GPIO from project config (add this in menuconfig via Kconfig).
   If you haven't added CONFIG_LIMIT_SWITCH_GPIO yet, replace it with a GPIO number:
     #define LIMIT_SWITCH_GPIO GPIO_NUM_9
*/
#define LIMIT_SWITCH_GPIO GPIO_NUM_6

/* Output pin for the burn wire activation
*/
#define GPIO_FIRE_EN GPIO_NUM_7

/* Active level for the limit switch input:
   - If wired to GND with pull-up: active is LOW (0)
   - If wired to 3.3V with pull-down: active is HIGH (1)
*/
#define LIMIT_SWITCH_ACTIVE_LEVEL 0

/* Simple debounce delay to avoid chatter on mechanical switches (ms) */
#define DEBOUNCE_MS 30

/* LED control */

static uint8_t limit_switch_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP
static led_strip_handle_t led_strip;

/* Turn addressable LED on/off based on limit_switch_state */
static void led_apply_state(void)
{
    if (limit_switch_state) {
        /* Pick a color when switch is active */
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
        .max_leds = 1, /* onboard: at least one LED */
    };

#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, /* 10 MHz */
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

    /* Start with LED off */
    limit_switch_state = 0;
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void led_apply_state(void)
{
    gpio_set_level(LED_GPIO, limit_switch_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

// Handle the GPIO pins for firing
static void configure_wire(void)
{
    // FIRE_EN output
    gpio_reset_pin(GPIO_FIRE_EN);
    gpio_set_direction(GPIO_FIRE_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_FIRE_EN, 0);
}

/* ----------------------------- Limit switch handling ----------------------------- */

static inline bool limit_switch_is_active(int level)
{
    // Return true if the limit switch level matches the active level
    return (level == LIMIT_SWITCH_ACTIVE_LEVEL);
}

// Configure the limit switch GPIO for input
static void configure_limit_switch(void)
{
    gpio_config_t limit_switch_config = {
        .pin_bit_mask = (1ULL << LIMIT_SWITCH_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Assuming switch is wired to GND
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&limit_switch_config));
}

/* Turn burn wire GPIO signal on/off based on limit_switch_state */
static void burn_wire_apply_state(void)
{
    if (limit_switch_state) {
        // Pulse FIRE_EN regardless of whether still held afterward
        ESP_LOGI(TAG, "Activating FIRE_EN");
        gpio_set_level(GPIO_FIRE_EN, 1);
    } else {
        ESP_LOGI(TAG, "Deactivating FIRE_EN");
        gpio_set_level(GPIO_FIRE_EN, 0);
    }
}


/* ----------------------------- app_main ----------------------------- */

void app_main(void)
{
    /* 1) Configure LED (same structure as the blink example) */
    configure_led();

    /* 2) Configure the limit switch GPIO for input */
    configure_limit_switch();

    /* 3) Configure the burn wire GPIO for output */
    configure_wire();

    /* 3) Check the limit switch level in a loop and apply to LED in a loop with delay */
    ESP_LOGI(TAG, "Started. LED_GPIO=%d (blink), FIRE_EN=%d, CMD=%d, active_level=%d",
            (int)LED_GPIO, (int)GPIO_FIRE_EN, (int)LIMIT_SWITCH_GPIO, LIMIT_SWITCH_ACTIVE_LEVEL);
    while(1) {
        int level = gpio_get_level(LIMIT_SWITCH_GPIO);
        limit_switch_state = limit_switch_is_active(level) ? 1 : 0;
        // If the limit switch is detected, send a signal through the GPIO_FIRE_EN and activate the LED
        burn_wire_apply_state();
        led_apply_state();
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    }
}