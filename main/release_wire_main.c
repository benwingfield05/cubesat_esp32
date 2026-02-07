// release_wire_main.c
//
// Protoype Overview:
// - Read a release command trigger from a GPIO input
// - Require three physical inhibit inputs to be in the "cleared" state
// - Output a current on one of the pins (FIRE_EN) to an external release mechanism (burn wire)
// - Once fired, enter LOCKOUT to check against accidental re-fires

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "sdkconfig.h"

// ----------------------- GPIO variables -----------------------

// Pin to receive release command trigger
#define GPIO_RELEASE_CMD      GPIO_NUM_6

// Three physical inhibits (based on spec but maybe get rid of this if project doesn't need it)
#define GPIO_INHIBIT_1        GPIO_NUM_2
#define GPIO_INHIBIT_2        GPIO_NUM_3
#define GPIO_INHIBIT_3        GPIO_NUM_4

// Pin to enable the release/firing mechanism with the 3v3 pin to heat the burn wire
#define GPIO_FIRE_EN          GPIO_NUM_7

// ----------------------- Levels for pins -----------------------

// Set to true to enable inhibit checking
#define INHIBITS_ACTIVE       false

// Common choice with pull-ups: inhibit pin connected to GND when "installed" -> reads 0.
// When removed/cleared, input floats high (pull-up) -> reads 1.
#define INHIBIT_CLEARED_LEVEL 1

// Release command trigger active level
// If using pull-up and switch to GND, active is 0. Otherwise set to 1.
#define RELEASE_ACTIVE_LEVEL  0

// ----------------------- Timing parameters -----------------------

// Debounce for inputs in ms (probably unnecessary but more customizable this way)
#define DEBOUNCE_MS           30
// Delay before allowing for firing after boot in ms
#define MIN_UPTIME_BEFORE_ARM_MS 2000
// Time to activate FIRE_EN output in ms
#define FIRE_PULSE_MS         3000


static const char *TAG = "burn_wire";
static int64_t s_boot_us;
static bool s_lockout = false;

// Read a GPIO pin with the debounce delay
static int read_gpio_debounced(gpio_num_t pin)
{
    int a = gpio_get_level(pin);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    int b = gpio_get_level(pin);
    (void)a;
    return b;
}

// Check that all inhibits are in the cleared state
static bool inhibits_cleared(void)
{
    int i1 = read_gpio_debounced(GPIO_INHIBIT_1);
    int i2 = read_gpio_debounced(GPIO_INHIBIT_2);
    int i3 = read_gpio_debounced(GPIO_INHIBIT_3);

    bool ok = (i1 == INHIBIT_CLEARED_LEVEL) &&
              (i2 == INHIBIT_CLEARED_LEVEL) &&
              (i3 == INHIBIT_CLEARED_LEVEL);

    if (!ok) {
        ESP_LOGW(TAG, "Inhibits not cleared: I1=%d I2=%d I3=%d", i1, i2, i3);
    }
    return ok;
}

// Check if minimum uptime before arming has elapsed
static bool min_uptime_elapsed(void)
{
    int64_t up_ms = (esp_timer_get_time() - s_boot_us) / 1000;
    return (up_ms >= MIN_UPTIME_BEFORE_ARM_MS);
}

// Configure all GPIOs used in the application
static void configure_gpio(void)
{
    // FIRE_EN output
    gpio_reset_pin(GPIO_FIRE_EN);
    gpio_set_direction(GPIO_FIRE_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_FIRE_EN, 0);

    // Rlelease command input
    gpio_config_t cmd = {
        .pin_bit_mask = 1ULL << GPIO_RELEASE_CMD,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (RELEASE_ACTIVE_LEVEL == 0) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (RELEASE_ACTIVE_LEVEL == 1) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cmd));

    // Inhibit inputs
    gpio_config_t inh = {
        .pin_bit_mask = (1ULL << GPIO_INHIBIT_1) |
                        (1ULL << GPIO_INHIBIT_2) |
                        (1ULL << GPIO_INHIBIT_3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (INHIBIT_CLEARED_LEVEL == 1) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (INHIBIT_CLEARED_LEVEL == 0) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&inh));
}

void app_main(void)
{
    // Record the boot time to check minimum uptime later
    s_boot_us = esp_timer_get_time();
    configure_gpio();

    ESP_LOGI(TAG, "Prototype polling loop started. FIRE_EN=OFF.");

    // Initialize state to inactive
    bool prev_active = false;

    // Main polling loop
    while (1) {
        // Poll release command
        int lvl = gpio_get_level(GPIO_RELEASE_CMD);
        bool active = (lvl == RELEASE_ACTIVE_LEVEL);

        // Check if the trigger transitioned from inactive to active
        bool trigger = (active && !prev_active);
        prev_active = active;

        // If trigger detected, process it
        if (trigger) {
            ESP_LOGI(TAG, "Release trigger detected");

            if (s_lockout) { // If in lockout, don't release
                ESP_LOGW(TAG, "LOCKOUT: ignoring trigger");
            // } else if (!min_uptime_elapsed()) { // If uptime too low, don't release
            //     ESP_LOGW(TAG, "Uptime too low, ignoring trigger");
            // } else if (INHIBITS_ACTIVE && !inhibits_cleared()) { // If inhibits are active and not cleared, don't release
            //     ESP_LOGW(TAG, "Inhibits not cleared, ignoring trigger");
            } else { // Fire at will
                // Final check on trigger
                int stable = read_gpio_debounced(GPIO_RELEASE_CMD);
                if (stable != RELEASE_ACTIVE_LEVEL) {
                    ESP_LOGW(TAG, "Trigger not stable after debounce check, not firing");
                } else {
                    // Activate FIRE_EN output
                    ESP_LOGI(TAG, "Asserting FIRE_EN for %d ms", FIRE_PULSE_MS);
                    // As long as the trigger is held, keep FIRE_EN on.
                    while (gpio_get_level(GPIO_RELEASE_CMD) == RELEASE_ACTIVE_LEVEL) {
                        gpio_set_level(GPIO_FIRE_EN, 1);
                        vTaskDelay(pdMS_TO_TICKS(FIRE_PULSE_MS));
                    }
                    gpio_set_level(GPIO_FIRE_EN, 0);

                    // Enter lockout to prevent re-fires
                    // s_lockout = true;
                    // ESP_LOGW(TAG, "Entered LOCKOUT (requires reset to clear)");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    }
}