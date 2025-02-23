#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simple struct for an RGB color in GRB order (WS2812 expects G->R->B)
 */
typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} led_color_t;

/**
 * @brief Initialize the LED strip on a given GPIO, with a specified number of WS2812 LEDs.
 *
 * @param[in] gpio     The data pin for the WS2812 LED strip
 * @param[in] num_leds The number of LEDs in the strip
 */
void LEDControl_init(int gpio, int num_leds);

/**
 * @brief Set all LEDs to the same color (in GRB format).
 */
void LEDControl_setBrightness(uint8_t brightness);

/**
 * @brief Set all LEDs to the same color (in GRB format).
 */
void LEDControl_setAll(led_color_t color);

/**
 * @brief Push the in-memory color buffer out to the physical LED strip via RMT.
 */
void LEDControl_update(void);

#ifdef __cplusplus
}
#endif
