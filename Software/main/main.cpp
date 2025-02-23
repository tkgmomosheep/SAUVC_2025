#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "math.h"
#include "driver/uart.h"

// Include your sensor and LED libraries
#include "MS5837.h"
#include "LEDControl.h"

// --- Global Definitions ---
static const char* TAG = "MAIN";

// We define two pins for the MS5837 (sda, scl), which are passed to sensor.init()
#define SDA0_Pin 4
#define SCL0_Pin 5

#define UART_TX_PIN GPIO_NUM_43
#define UART_RX_PIN GPIO_NUM_44
#define UART_PORT   UART_NUM_0

// Thruster settings
static const int thrusterCount = 8;
static const uint8_t middle_power = 0x80;
static int motor_direction[thrusterCount] = {1,1,1,1,1,1,1,1};
static uint8_t thruster_pwm[thrusterCount] = {
    middle_power, middle_power, middle_power, middle_power,
    middle_power, middle_power, middle_power, middle_power
};
static int thruster_pin_no[thrusterCount] = {15, 16, 17, 18, 39, 40, 41, 42};

// LEDC parameters for thrusters
const int pwmFreq = 50;  // 50 Hz for servos
const int pwmResolution = LEDC_TIMER_14_BIT;
int ledc_channels[thrusterCount] = {0,1,2,3,4,5,6,7};

// Timing and safety
static unsigned long lastCommandMillis = 0;
static const unsigned long commandTimeout = 1000;

// Error state for heartbeat LED
enum ErrorState { ERROR_NONE = 0, ERROR_SENSOR_INIT, ERROR_OTHER };
volatile ErrorState errorState = ERROR_NONE;

// Global data for the MS5837 sensor reading
static float currentPressure = 0.0;

// Create one **global** MS5837 object
static MS5837 sensor;

// Mutexes for protecting shared data
SemaphoreHandle_t thrusterMutex;
SemaphoreHandle_t sensorMutex;

/************************************************************************************/
// (1) Sensor Reading Task
void TaskSensorRead(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        // 1) read sensor
        sensor.read();
        float pressure = sensor.pressure(); // same as old code

        // 2) store in currentPressure under sensorMutex
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentPressure = pressure;
            xSemaphoreGive(sensorMutex);
        }

        // ~20 Hz
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/************************************************************************************/
// Communication Task
void TaskCommunication(void *pvParameters)
{
    (void) pvParameters;
    for (;;) {
        // Attempt to read up to 32 bytes from UART
        uint8_t buffer[32];
        int len = uart_read_bytes(UART_PORT, buffer, sizeof(buffer), 10 / portTICK_PERIOD_MS);

        if (len > 0) {
            // parse
            if (len >= thrusterCount + 2 &&
                buffer[0] == 0xFF &&
                buffer[thrusterCount + 1] == 0xAA)
            {
                // update thruster_pwm under thrusterMutex
                if (xSemaphoreTake(thrusterMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    for (int i=0; i<thrusterCount; i++) {
                        thruster_pwm[i] = buffer[i+1];
                    }
                    xSemaphoreGive(thrusterMutex);
                }
                // Mark last command
                lastCommandMillis = (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            }
        }

        uint8_t localThrusters[thrusterCount];
        if (xSemaphoreTake(thrusterMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (int i=0; i<thrusterCount; i++) {
                localThrusters[i] = thruster_pwm[i];
            }
            xSemaphoreGive(thrusterMutex);
        }
        // copy pressure -> local
        float localPressure = 0.0;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localPressure = currentPressure;
            xSemaphoreGive(sensorMutex);
        }
        char sendBuffer[128];
        int sendLen = snprintf(sendBuffer, sizeof(sendBuffer),
                               "T0:%d|T1:%d|T2:%d|T3:%d|T4:%d|T5:%d|T6:%d|T7:%d|P:%.2f\r\n",
                               localThrusters[0], localThrusters[1], localThrusters[2], localThrusters[3],
                               localThrusters[4], localThrusters[5], localThrusters[6], localThrusters[7],
                               localPressure);
        // send the buffer
        uart_tx_chars(UART_PORT, sendBuffer, sendLen);
        // small delay
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// Thruster Control Task
void TaskThrusterControl(void *pvParameters)
{
    (void)pvParameters;
    for(;;){
        unsigned long now = (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        // if no command for 'commandTimeout' ms => reset thrusters
        if((now - lastCommandMillis) >= commandTimeout){
            if(xSemaphoreTake(thrusterMutex, pdMS_TO_TICKS(10)) == pdTRUE){
                for(int i=0;i<thrusterCount;i++){
                    thruster_pwm[i] = middle_power;
                }
                xSemaphoreGive(thrusterMutex);
            }
        }

        // read thruster_pwm local
        uint8_t localPWM[thrusterCount];
        if(xSemaphoreTake(thrusterMutex, pdMS_TO_TICKS(10)) == pdTRUE){
            for(int i=0;i<thrusterCount;i++){
                localPWM[i] = thruster_pwm[i];
            }
            xSemaphoreGive(thrusterMutex);
        }

        // same approach: compute pulse => map to LEDC
        for(int i=0; i<thrusterCount; i++){
            float power = float(localPWM[i] - middle_power)/128.0f;
            int calcPulse = 1500 + int(400*power*motor_direction[i]);
            if(calcPulse<1100) calcPulse=1100;
            if(calcPulse>1900) calcPulse=1900;
            int ledc_val = int(0.82f*calcPulse+65);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channels[i], ledc_val);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channels[i]);
        }

        // ~ 1 ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/************************************************************************************/
// (4) Heartbeat LED Task
void TaskHeartbeatLED(void *pvParameters) {
    (void) pvParameters;
    // Initialize RMT-based LED on pin 38 with 1 LED
    LEDControl_init(38, 1);
    // Set brightness to 5% (5% of 255 ≈ 13)
    LEDControl_setBrightness(13);

    for (;;) {
        if (errorState == ERROR_SENSOR_INIT) {
            // Show flashing blue
            led_color_t blue = { .g = 0, .r = 0, .b = 255 };
            LEDControl_setAll(blue);
            LEDControl_update();
            vTaskDelay(pdMS_TO_TICKS(250));
            led_color_t off = { 0, 0, 0 };
            LEDControl_setAll(off);
            LEDControl_update();
            vTaskDelay(pdMS_TO_TICKS(250));
        } else if (errorState == ERROR_OTHER) {
            // Show flashing red
            led_color_t red = { .g = 0, .r = 255, .b = 0 };
            LEDControl_setAll(red);
            LEDControl_update();
            vTaskDelay(pdMS_TO_TICKS(250));
            led_color_t off = { 0, 0, 0 };
            LEDControl_setAll(off);
            LEDControl_update();
            vTaskDelay(pdMS_TO_TICKS(250));
        } else if (errorState == ERROR_NONE) {
            // Pulse green
            for (int b = 0; b <= 255; b += 5) {
                led_color_t green = { (uint8_t)b, 0, 0 };
                LEDControl_setAll(green);
                LEDControl_update();
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            for (int b = 255; b >= 0; b -= 5) {
                led_color_t green = { (uint8_t)b, 0, 0 };
                LEDControl_setAll(green);
                LEDControl_update();
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}


/************************************************************************************/
// (5) app_main (Entry Point)
extern "C" void app_main(void) {
    // esp_log_level_set("*", ESP_LOG_DEBUG);

    thrusterMutex = xSemaphoreCreateMutex();
    sensorMutex   = xSemaphoreCreateMutex();

    // 1) Basic UART config
    const uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0));

    // Configure LEDC for thrusters
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)pwmResolution,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = pwmFreq,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < thrusterCount; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = thruster_pin_no[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = {
                .output_invert = 1
            },
        };
        ledc_channel_config(&ch_cfg);
    }

    lastCommandMillis = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // Initialize the MS5837 sensor with custom pins
    ESP_LOGI(TAG, "Initializing MS5837 sensor on SDA=%d, SCL=%d", SDA0_Pin, SCL0_Pin);
    // Keep trying to initialize the sensor until it succeeds
    while (!sensor.init(SDA0_Pin, SCL0_Pin)) {
        ESP_LOGE(TAG, "Sensor init failed!");
        errorState = ERROR_SENSOR_INIT;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    sensor.setFluidDensity(997);
    errorState = ERROR_NONE;
    ESP_LOGI(TAG, "MS5837 sensor initialized successfully");

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(TaskSensorRead, "SensorRead", 4096, NULL, 1, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(TaskCommunication, "CommTask", 4096, NULL, 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(TaskThrusterControl, "ThrusterCtrl", 2048, NULL, 1, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(TaskHeartbeatLED,  "HeartbeatLED", 4096, NULL, 1, NULL, APP_CPU_NUM);
}
