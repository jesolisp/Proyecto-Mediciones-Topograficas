/*
 * Dr. J.E. Solis-Perez <jsolisp@unam.mx>
 * Created: May 23, 2024
 * Modified: June 23, 2024
 */

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"

#define ADC1_CHAN7 ADC_CHANNEL_7 /* GPIO 35 for Sensor 1 */
#define ADC1_CHAN4 ADC_CHANNEL_4 /* GPIO 32 for Sensor 2 */
#define ADC1_CHAN5 ADC_CHANNEL_5 /* GPIO 33 for Sensor 3 */
#define ADC_ATTEN ADC_ATTEN_DB_11
#define N_SENSORS 3

#define GPIO_SENSOR1_STATE GPIO_NUM_25 /* GPIO pin for Sensor 1 state */
#define GPIO_SENSOR2_STATE GPIO_NUM_26 /* GPIO pin for Sensor 2 state */
#define GPIO_SENSOR3_STATE GPIO_NUM_27 /* GPIO pin for Sensor 3 state */

static const char *TAG = "System v0.1";

adc_oneshot_unit_handle_t adc1_handle;
static int adc_raws[N_SENSORS] = {0, 0, 0};
static float adc_voltages[N_SENSORS] = {0.0, 0.0, 0.0};
static float distances[N_SENSORS] = {0.0, 0.0, 0.0};
static bool sensor_states[N_SENSORS] = {true, true, true}; // States for each sensor

/* Coefficients for the distance conversion polynomials */
static const float coefficients[N_SENSORS][6] = {
    {0.43845, -3.1136, 6.6372, -5.4496, 35.144, 0.53767}, // Sensor 1
    {0.43845, -3.1136, 6.6372, -5.4496, 35.144, 0.53767}, // Sensor 2
    {0.43845, -3.1136, 6.6372, -5.4496, 35.144, 0.53767}  // Sensor 3
};

void gpio_initialize() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_SENSOR1_STATE) | (1ULL << GPIO_SENSOR2_STATE) | (1ULL << GPIO_SENSOR3_STATE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void update_sensor_states() {
    sensor_states[0] = gpio_get_level(GPIO_SENSOR1_STATE);
    sensor_states[1] = gpio_get_level(GPIO_SENSOR2_STATE);
    sensor_states[2] = gpio_get_level(GPIO_SENSOR3_STATE);
}

esp_err_t adc_initialize(){
    /* ADC1 Init */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_new_unit(&init_config, &adc1_handle);

    /* ADC1 Config */
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };

    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN7, &config);
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN4, &config);
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN5, &config);

    return ESP_OK;
}

esp_err_t get_adc_value(adc_oneshot_unit_handle_t handle, adc_channel_t chan, int *out_raw){
    return adc_oneshot_read(handle, chan, out_raw);
}

void calculate_distance(int sensor_index, float voltage) {
    const float *coeff = coefficients[sensor_index];
    distances[sensor_index] = coeff[0]*voltage*voltage*voltage*voltage*voltage +
                               coeff[1]*voltage*voltage*voltage*voltage +
                               coeff[2]*voltage*voltage*voltage +
                               coeff[3]*voltage*voltage +
                               coeff[4]*voltage +
                               coeff[5];
}

void read_sensor(int sensor_index, adc_channel_t channel) {
    get_adc_value(adc1_handle, channel, &adc_raws[sensor_index]);
    adc_voltages[sensor_index] = adc_raws[sensor_index] * 3.3 / 4095.0;
    calculate_distance(sensor_index, adc_voltages[sensor_index]);
    ESP_LOGI(TAG, "Sensor %d Distance: %.2f mm", sensor_index + 1, distances[sensor_index]);
}

void app_main(void){
    adc_initialize(); /* ADC initialization */
    gpio_initialize(); /* GPIO initialization */

    while(1){
        update_sensor_states(); /* Update sensor states from GPIO */

        if (sensor_states[0]) {
            read_sensor(0, ADC1_CHAN7);
        } else {
            ESP_LOGI(TAG, "Sensor 1 is disabled");
        }

        if (sensor_states[1]) {
            read_sensor(1, ADC1_CHAN4);
        } else {
            ESP_LOGI(TAG, "Sensor 2 is disabled");
        }

        if (sensor_states[2]) {
            read_sensor(2, ADC1_CHAN5);
        } else {
            ESP_LOGI(TAG, "Sensor 3 is disabled");
        }

        ESP_LOGI(TAG, "Entering deep sleep for 5 minutes...");
        esp_deep_sleep(300000000); // 5 minutes in microseconds
        // For 12 hours, change to: esp_deep_sleep(43200000000);
    }
}
