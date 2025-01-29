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
#include "esp_sleep.h"

#define ADC1_CHAN7 ADC_CHANNEL_7 /* GPIO 35 for Sensor 1 */
#define ADC1_CHAN4 ADC_CHANNEL_4 /* GPIO 32 for Sensor 2 */
#define ADC1_CHAN5 ADC_CHANNEL_5 /* GPIO 33 for Sensor 3 */
#define ADC_ATTEN ADC_ATTEN_DB_11
#define N_SENSORS 3
#define BUF_SIZE 100

static const char *TAG = "System v0.1";

adc_oneshot_unit_handle_t adc1_handle;
static int adc_raws[N_SENSORS] = {0, 0, 0};
static float adc_voltages[N_SENSORS] = {0.0, 0.0, 0.0};
static float distances[N_SENSORS] = {0.0, 0.0, 0.0};

/* Coefficients for the distance conversion polynomials */
static const float coefficients[N_SENSORS][6] = {
    {0.43845, -3.1136, 6.6372, -5.4496, 35.144, 0.53767}, // Sensor 1
    {0.43845, -3.1136, 6.6372, -5.4496, 35.144, 0.53767}, // Sensor 2
    {0.43845, -3.1136, 6.6372, -5.4496, 35.144, 0.53767}  // Sensor 3
};

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

void app_main(void){
    adc_initialize(); /* ADC initialization */

    while(1){
        /* Sensor 1 */
        get_adc_value(adc1_handle, ADC1_CHAN7, &adc_raws[0]);
        adc_voltages[0] = adc_raws[0] * 3.3 / 4095.0;
        calculate_distance(0, adc_voltages[0]);
        ESP_LOGI(TAG, "Sensor 1 Distance: %.2f mm", distances[0]);

        /* Sensor 2 */
        get_adc_value(adc1_handle, ADC1_CHAN4, &adc_raws[1]);
        adc_voltages[1] = adc_raws[1] * 3.3 / 4095.0;
        calculate_distance(1, adc_voltages[1]);
        ESP_LOGI(TAG, "Sensor 2 Distance: %.2f mm", distances[1]);

        /* Sensor 3 */
        get_adc_value(adc1_handle, ADC1_CHAN5, &adc_raws[2]);
        adc_voltages[2] = adc_raws[2] * 3.3 / 4095.0;
        calculate_distance(2, adc_voltages[2]);
        ESP_LOGI(TAG, "Sensor 3 Distance: %.2f mm", distances[2]);

        /* Sleep the ESP32 */
        ESP_LOGI(TAG, "Entering deep sleep for 5 minutes...");
        esp_deep_sleep(300000000); // 5 minutes in microseconds
        // For 12 hours, change to: esp_deep_sleep(43200000000);
    }
}
