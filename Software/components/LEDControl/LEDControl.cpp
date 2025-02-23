#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "LEDControl.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"  // for rmt_new_simple_encoder, etc.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LEDControl";

// We'll store some static data in this component:
static rmt_channel_handle_t s_led_chan     = NULL; // RMT TX channel
static rmt_encoder_handle_t s_simple_enc   = NULL; // Simple callback-based encoder
static uint8_t *s_led_buffer               = NULL; // Buffer storing colors in GRB order
static int s_led_count                     = 0;

// Global brightness (0=off, 255=full brightness). Default is 255.
static uint8_t s_brightness                = 255;

// We'll use a 10MHz resolution (1 tick = 0.1us) for driving WS2812 LEDs.
#define RMT_LED_STRIP_RESOLUTION_HZ (10 * 1000 * 1000)

// Timeout for waiting for transmission to complete
#define FRAME_REFRESH_TIMEOUT_MS  100

// ---------------------------------------------------------------------------
// Define WS2812 timing parameters (in ticks)
// T0H = 0.3us, T0L = 0.9us, T1H = 0.9us, T1L = 0.3us
static const rmt_symbol_word_t s_ws2812_zero = {
    .duration0 = (uint16_t)(0.3f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000),
    .level0    = 1,
    .duration1 = (uint16_t)(0.9f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000),
    .level1    = 0,
};

static const rmt_symbol_word_t s_ws2812_one = {
    .duration0 = (uint16_t)(0.9f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000),
    .level0    = 1,
    .duration1 = (uint16_t)(0.3f * RMT_LED_STRIP_RESOLUTION_HZ / 1000000),
    .level1    = 0,
};

static const rmt_symbol_word_t s_ws2812_reset = {
    .duration0 = (uint16_t)((50 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000) / 2),
    .level0    = 0,
    .duration1 = (uint16_t)((50 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000) / 2),
    .level1    = 0,
};

// ---------------------------------------------------------------------------
// Callback function: encode LED data into RMT symbols for WS2812
static size_t led_strip_callback(const void *data, size_t data_size,
                                 size_t symbols_written, size_t symbols_free,
                                 rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    if (symbols_free < 8) {
        return 0; // not enough space to encode one byte
    }

    size_t byte_pos = symbols_written / 8;
    const uint8_t *p = (const uint8_t *)data;

    if (byte_pos < data_size) {
        uint8_t b = p[byte_pos];
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            if (b & bitmask) {
                symbols[symbol_pos++] = s_ws2812_one;
            } else {
                symbols[symbol_pos++] = s_ws2812_zero;
            }
        }
        return symbol_pos; // 8 symbols for one byte
    } else {
        symbols[0] = s_ws2812_reset;
        *done = true;
        return 1; // one symbol for reset
    }
}

// ---------------------------------------------------------------------------
// LEDControl_init: Initialize the LED strip
void LEDControl_init(int gpio, int num_leds)
{
    ESP_LOGI(TAG, "Initialize LED strip on GPIO=%d, num_leds=%d", gpio, num_leds);
    s_led_count = num_leds;

    // Allocate or reallocate LED buffer
    if (s_led_buffer) {
        free(s_led_buffer);
        s_led_buffer = NULL;
    }
    s_led_buffer = (uint8_t *)calloc(num_leds * 3, sizeof(uint8_t));
    if (!s_led_buffer) {
        ESP_LOGE(TAG, "No memory for LED buffer");
        return;
    }

    // Create RMT TX channel
    rmt_tx_channel_config_t tx_chan_cfg = {
        .gpio_num         = (gpio_num_t)gpio,
        .clk_src          = RMT_CLK_SRC_DEFAULT,
        .resolution_hz    = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols= 64,
        .trans_queue_depth= 4,
        .intr_priority    = 0,
        .flags = {
            .with_dma = false
        },
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &s_led_chan));

    // Create a simple callback-based encoder
    rmt_encoder_handle_t simple_enc = NULL;
    rmt_simple_encoder_config_t simple_enc_cfg = {
        .callback = led_strip_callback,
        .min_chunk_size = 64,
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_enc_cfg, &simple_enc));
    s_simple_enc = simple_enc;

    // Enable the channel
    ESP_ERROR_CHECK(rmt_enable(s_led_chan));

    // Default brightness to full (255)
    s_brightness = 255;

    ESP_LOGI(TAG, "LEDControl init done");
}

// ---------------------------------------------------------------------------
// LEDControl_setAll: Fill the LED buffer with the specified GRB color.
void LEDControl_setAll(led_color_t color)
{
    if (!s_led_buffer) {
        ESP_LOGW(TAG, "LED buffer not allocated");
        return;
    }
    for (int i = 0; i < s_led_count; i++) {
        // Store in GRB order
        s_led_buffer[i * 3 + 0] = color.g;
        s_led_buffer[i * 3 + 1] = color.r;
        s_led_buffer[i * 3 + 2] = color.b;
    }
}

// ---------------------------------------------------------------------------
// LEDControl_setBrightness: Set the brightness (0 to 255)
// 0 = off, 255 = full brightness.
void LEDControl_setBrightness(uint8_t brightness)
{
    s_brightness = brightness;
    ESP_LOGI(TAG, "LED brightness set to %d", s_brightness);
}

// ---------------------------------------------------------------------------
// LEDControl_update: Transmit the LED buffer with brightness scaling via RMT
void LEDControl_update(void)
{
    if (!s_led_chan || !s_simple_enc || !s_led_buffer) {
        ESP_LOGE(TAG, "LEDControl not properly initialized");
        return;
    }

    // Allocate a temporary buffer for transmission with brightness scaling.
    size_t data_size = s_led_count * 3;
    uint8_t *tx_buffer = (uint8_t *)malloc(data_size);
    if (!tx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate tx_buffer");
        return;
    }

    // Scale each LED's color by the brightness factor.
    // We assume s_led_buffer stores raw values (0-255). Multiply each value by (s_brightness/255).
    float scale = s_brightness / 255.0f;
    for (int i = 0; i < data_size; i++) {
        float scaled = s_led_buffer[i] * scale;
        if (scaled > 255) {
            scaled = 255;
        }
        tx_buffer[i] = (uint8_t)scaled;
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no looping
    };

    ESP_ERROR_CHECK(rmt_transmit(s_led_chan, s_simple_enc, tx_buffer, data_size, &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_led_chan, pdMS_TO_TICKS(FRAME_REFRESH_TIMEOUT_MS)));

    free(tx_buffer);
}
