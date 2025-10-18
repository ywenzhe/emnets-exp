#include "ztimer.h"
#include "ledcontroller.hh"
#include "periph/gpio.h"  
#include <stdio.h>
/**
 * Initialize the RGB pins as output mode, and initially, the lights should be off.
 * gpio_init(gpio_pin, GPIO_OUT);
 * gpio_write(pin, 0);
 */
LEDController::LEDController(uint8_t gpio_r, uint8_t gpio_g, uint8_t gpio_b) {
    printf("LED Controller initialized with (RGB: GPIO%d, GPIO%d, GPIO%d)\n", gpio_r, gpio_g, gpio_b);

    // Store GPIO pins
    led_gpio[0] = gpio_r;
    led_gpio[1] = gpio_g;
    led_gpio[2] = gpio_b;

    // Initialize RGB pins as output mode
    gpio_init(led_gpio[0], GPIO_OUT);
    gpio_init(led_gpio[1], GPIO_OUT);
    gpio_init(led_gpio[2], GPIO_OUT);

    // Initially turn off all LEDs
    gpio_write(led_gpio[0], 0);
    gpio_write(led_gpio[1], 0);
    gpio_write(led_gpio[2], 0);

    // Initialize RGB values to off
    rgb[0] = 0;
    rgb[1] = 0;
    rgb[2] = 0;
}

/**
 * Implement a light that displays at least 5 status colors through the RGB three pins.
 * ------------------------------------------------------------
 * @note Method 1
 * Utilizes the gpio_write function to set the GPIO pin connected to the LED to the current LED state.
 * void gpio_write(uint8_t pin, int value);
 * @param pin The GPIO pin connected to the LED.
 * @param value The value to set the GPIO pin to (0 for LOW, 1 for HIGH).
 * ------------------------------------------------------------
 * @note Method 2
 * Uses the gpio_set and gpio_clear functions to set the GPIO pin connected to the LED to the current LED state.
 * void gpio_set(uint8_t pin); void gpio_clear(uint8_t pin);
 * @param pin The GPIO pin connected to the LED.
 */
void LEDController::change_led_color(uint8_t color) {
    // Set RGB values based on color enum
    switch (color) {
    case COLOR_NONE:
        rgb[0] = 0;  // R
        rgb[1] = 0;  // G
        rgb[2] = 0;  // B
        break;
    case COLOR_RED:
        rgb[0] = 1;  // R
        rgb[1] = 0;  // G
        rgb[2] = 0;  // B
        break;
    case COLOR_GREEN:
        rgb[0] = 0;  // R
        rgb[1] = 1;  // G
        rgb[2] = 0;  // B
        break;
    case COLOR_YELLO:
        rgb[0] = 1;  // R
        rgb[1] = 1;  // G
        rgb[2] = 0;  // B
        break;
    case COLOR_BLUE:
        rgb[0] = 0;  // R
        rgb[1] = 0;  // G
        rgb[2] = 1;  // B
        break;
    case COLOR_MAGENTA:
        rgb[0] = 1;  // R
        rgb[1] = 0;  // G
        rgb[2] = 1;  // B
        break;
    case COLOR_CYAN:
        rgb[0] = 0;  // R
        rgb[1] = 1;  // G
        rgb[2] = 1;  // B
        break;
    case COLOR_WHITE:
        rgb[0] = 1;  // R
        rgb[1] = 1;  // G
        rgb[2] = 1;  // B
        break;
    default:
        // Default to off for unknown colors
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
        break;
    }

    // Apply RGB values to GPIO pins
    gpio_write(led_gpio[0], rgb[0]);
    gpio_write(led_gpio[1], rgb[1]);
    gpio_write(led_gpio[2], rgb[2]);
}

