#include "MS5837.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char* TAG = "MS5837";

// I2C device address for MS5837
#define MS5837_ADDR               0x76  
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

const float MS5837::Pa   = 100.0f;
const float MS5837::bar  = 0.001f;
const float MS5837::mbar = 1.0f;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;

// We'll store static bus/device handles so we only create them once
static i2c_master_bus_handle_t s_ms5837_bus = NULL;
static i2c_master_dev_handle_t s_ms5837_dev = NULL;

// We'll reduce queue depth to 4 for synchronous usage
static i2c_master_bus_config_t s_i2c_config = {
    .i2c_port            = I2C_NUM_0,
    .sda_io_num          = GPIO_NUM_NC,   // assigned in init(...)
    .scl_io_num          = GPIO_NUM_NC,   // assigned in init(...)
    .clk_source          = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt   = 7,
    // .intr_priority       = 3,
    // .trans_queue_depth   = 10,   // usually enough for synchronous usage
    .flags = {
        .enable_internal_pullup = 1,
        // .allow_pd = 0,
    },
};

static i2c_device_config_t s_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = MS5837_ADDR,
    .scl_speed_hz    = 100000, // 100 kHz I2C
    // .scl_wait_us     = 0,
    // .flags = {
        // .disable_ack_check = 0, // If your hardware is good, you can set =0 to check ACK
    // },
};

MS5837::MS5837() {
    fluidDensity = 1029;
}

bool MS5837::init(int sda_pin, int scl_pin) {
    esp_err_t ret;

    // Create bus if not created
    if (s_ms5837_bus == NULL) {
        s_i2c_config.sda_io_num = (gpio_num_t)sda_pin;
        s_i2c_config.scl_io_num = (gpio_num_t)scl_pin;
        ret = i2c_new_master_bus(&s_i2c_config, &s_ms5837_bus);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
            return false;
        }
    }
    // Add device if not added
    if (s_ms5837_dev == NULL) {
        ret = i2c_master_bus_add_device(s_ms5837_bus, &s_dev_cfg, &s_ms5837_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
            return false;
        }
    }

    // (1) Send reset command (write only)
    uint8_t cmd = MS5837_RESET;
    ret = i2c_master_transmit(s_ms5837_dev, &cmd, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reset cmd failed: %s", esp_err_to_name(ret));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // wait after reset

    // (2) Read calibration values (7 entries) using i2c_master_transmit_receive
    for (uint8_t i = 0; i < 7; i++) {
        // The PROM address we want to read
        uint8_t prom_addr = (uint8_t)(MS5837_PROM_READ + i*2);
        // We do a single transaction: write 1 byte => [prom_addr], read 2 bytes
        uint8_t rx_data[2];
        esp_err_t ret2 = i2c_master_transmit_receive(
                            s_ms5837_dev,
                            &prom_addr, 1,   // write buffer
                            rx_data, 2,      // read buffer
                            -1               // default timeout
                         );
        if (ret2 != ESP_OK) {
            ESP_LOGE(TAG, "PROM read i=%d failed: %s", i, esp_err_to_name(ret2));
            return false;
        }
        // Combine the two bytes into a single 16-bit value
        C[i] = (rx_data[0] << 8) | rx_data[1];
    }

    // Check CRC
    uint8_t crcRead       = C[0] >> 12;
    uint8_t crcCalculated = crc4(C);
    if (crcCalculated != crcRead) {
        ESP_LOGE(TAG, "CRC mismatch: calc=%d read=%d", crcCalculated, crcRead);
        return false;
    }
    return true;
}

void MS5837::setModel(uint8_t model) {
    _model = model;
}

void MS5837::setFluidDensity(float density) {
    fluidDensity = density;
}

void MS5837::read() {
    if (!s_ms5837_dev) {
        ESP_LOGE(TAG, "Device handle not initialized!");
        return;
    }
    esp_err_t ret;

    // (A) Request D1 conversion (just write 1 byte)
    uint8_t conv_cmd_d1 = MS5837_CONVERT_D1_8192;
    ret = i2c_master_transmit(s_ms5837_dev, &conv_cmd_d1, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "D1 convert cmd fail: %s", esp_err_to_name(ret));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // (B) Read D1 in a single transaction:
    //    We write [MS5837_ADC_READ], read 3 bytes
    uint8_t read_cmd = MS5837_ADC_READ;
    uint8_t rx_buf[3];
    ret = i2c_master_transmit_receive(s_ms5837_dev,
                                      &read_cmd, 1,  // write
                                      rx_buf, 3,     // read
                                      -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read D1: %s", esp_err_to_name(ret));
        return;
    }
    D1 = 0;
    D1 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | rx_buf[2];
    // (C) Request D2 conversion
    uint8_t conv_cmd_d2 = MS5837_CONVERT_D2_8192;
    ret = i2c_master_transmit(s_ms5837_dev, &conv_cmd_d2, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "D2 convert cmd fail: %s", esp_err_to_name(ret));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // (D) Read D2 in single transaction
    read_cmd = MS5837_ADC_READ;
    ret = i2c_master_transmit_receive(s_ms5837_dev,
                                      &read_cmd, 1,
                                      rx_buf, 3,
                                      -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read D2: %s", esp_err_to_name(ret));
        return;
    }
    D2 = 0;
    D2 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | rx_buf[2];

    // Now do the calculation
    calculate();
}

void MS5837::calculate() {
    // Same code as before
    int32_t dT = D2 - uint32_t(C[5]) * 256l;
    int64_t SENS = 0;
    int64_t OFF  = 0;
    int32_t SENSi= 0;
    int32_t OFFi = 0;
    int32_t Ti   = 0;
    int64_t OFF2 = 0;
    int64_t SENS2= 0;

    
    if (_model == MS5837_02BA) {
        SENS = int64_t(C[1]) * 65536l + (int64_t(C[3]) * dT) / 128l;
        OFF  = int64_t(C[2]) * 131072l + (int64_t(C[4]) * dT) / 64l;
        P    = (D1 * SENS / 2097152l - OFF) / 32768l;
    } else {
        SENS = int64_t(C[1]) * 32768l + (int64_t(C[3]) * dT) / 256l;
        OFF  = int64_t(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;
        P    = (D1 * SENS / 2097152l - OFF) / 8192l;
    }
    
    TEMP = 2000l + int64_t(dT) * C[6] / 8388608LL;
    if (_model == MS5837_02BA) {
        if ((TEMP / 100) < 20) {
            Ti   = (11 * int64_t(dT) * int64_t(dT)) / 34359738368LL;
            OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            SENSi= (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
        }
    } else {
        if ((TEMP / 100) < 20) {
            Ti   = (3 * int64_t(dT) * int64_t(dT)) / 8589934592LL;
            OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
            SENSi= (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            if ((TEMP / 100) < -15) {
                OFFi  += 7 * (TEMP + 1500l) * (TEMP + 1500l);
                SENSi += 4 * (TEMP + 1500l) * (TEMP + 1500l);
            }
        } else if ((TEMP / 100) >= 20) {
            Ti   = 2 * (dT * dT) / 137438953472LL;
            OFFi = (TEMP - 2000) * (TEMP - 2000) / 16;
            SENSi= 0;
        }
    }
    
    OFF2  = OFF - OFFi;
    SENS2 = SENS - SENSi;
    TEMP  = TEMP - Ti;
    if (_model == MS5837_02BA) {
        P = ((D1 * SENS2) / 2097152l - OFF2) / 32768l;
    } else {
        P = ((D1 * SENS2) / 2097152l - OFF2) / 8192l;
    }
}

float MS5837::pressure(float conversion) {
    if (_model == MS5837_02BA)
        return P * conversion / 100.0f;
    else
        return P * conversion / 10.0f;
}

float MS5837::temperature() {
    return TEMP / 100.0f;
}

float MS5837::depth() {
    return (pressure(Pa) - 101300) / (fluidDensity * 9.80665);
}

float MS5837::altitude() {
    return (1 - pow((pressure() / 1013.25), 0.190284)) * 145366.45 * 0.3048;
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;
    n_prom[0] &= 0x0FFF;
    n_prom[7] = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (i % 2 == 1)
            n_rem ^= (n_prom[i >> 1] & 0x00FF);
        else
            n_rem ^= (n_prom[i >> 1] >> 8);
        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            n_rem = (n_rem & 0x8000) ? ((n_rem << 1) ^ 0x3000) : (n_rem << 1);
        }
    }
    n_rem = (n_rem >> 12) & 0x000F;
    return n_rem;
}
