#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "System v0.1";

#define ADC1_CHAN7 ADC_CHANNEL_7 /* GPIO 35 for Sensor 1 */
#define ADC1_CHAN4 ADC_CHANNEL_4 /* GPIO 32 for Sensor 2 */
#define ADC1_CHAN5 ADC_CHANNEL_5 /* GPIO 33 for Sensor 3 */
#define ADC_ATTEN ADC_ATTEN_DB_11
#define N_SENSORS 3

#define GPIO_SENSOR1_STATE GPIO_NUM_25 /* GPIO pin for Sensor 1 state */
#define GPIO_SENSOR2_STATE GPIO_NUM_26 /* GPIO pin for Sensor 2 state */
#define GPIO_SENSOR3_STATE GPIO_NUM_27 /* GPIO pin for Sensor 3 state */

// Definición de etiquetas y pines I2C
#define I2C_MASTER_SCL_IO           22      // Pin para SCL
#define I2C_MASTER_SDA_IO           21      // Pin para SDA
#define I2C_MASTER_FREQ_HZ          100000  // Frecuencia del bus I2C
#define I2C_MASTER_PORT             I2C_NUM_0
#define DS1307_ADDR                 0x68    // Dirección del RTC DS1307

#define PIN_TXD_UART_1 1
#define PIN_RXD_UART_1 3
#define PIN_TXD_UART_2 17 /* Connect to RX pin in LoRa board */
#define PIN_RXD_UART_2 16 /* Connect to TX pin in LoRa board */
#define PIN_RTS UART_PIN_NO_CHANGE
#define PIN_CTS UART_PIN_NO_CHANGE

#define UART_1_PORT UART_NUM_1
#define UART_2_PORT UART_NUM_2
#define UART_BAUD_RATE  115200
#define UART_BAUD_RATE_LORA 9600
#define ECHO_TASK_STACK_SIZE 2048

#define BUF_SIZE 1024

#define EXAMPLE_MAX_CHAR_SIZE 64
#define MOUNT_POINT "/sdcard"

// Pines para SPI (asignación explícita)
#define PIN_NUM_MISO 19  // GPIO19
#define PIN_NUM_MOSI 23  // GPIO23
#define PIN_NUM_CLK  18  // GPIO18
#define PIN_NUM_CS    5  // GPIO5

uint8_t hours = 0, minutes = 0, seconds = 0, date = 0, month = 0, year = 0;

adc_oneshot_unit_handle_t adc1_handle;
static int adc_raws[N_SENSORS] = {0, 0, 0};
static float adc_voltages[N_SENSORS] = {0.0, 0.0, 0.0};
static float distances[N_SENSORS] = {0.0, 0.0, 0.0};
static bool sensor_states[N_SENSORS] = {true, true, true}; // States for each sensor

// Coefficients for the distance conversion polynomials
static const float coefficients[N_SENSORS][6] = {
    {1.4985,  -13.4317, 42.569, -58.6858, 64.9932, 1.3185}, // Sensor 1
    {1.4985,  -13.4317, 42.569, -58.6858, 64.9932, 1.3185}, // Sensor 2
    {1.4985,  -13.4317, 42.569, -58.6858, 64.9932, 1.3185}  // Sensor 3
};

//Uart LoRa
esp_err_t uart1_initialization(){
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int intr_alloc_flags = 0;
    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(UART_1_PORT, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_1_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_1_PORT, PIN_TXD_UART_1, PIN_RXD_UART_1, PIN_RTS, PIN_CTS));

    return ESP_OK;
}

esp_err_t uart2_initialization(){
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE_LORA,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int intr_alloc_flags = 0;
    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(UART_2_PORT, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_2_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_2_PORT, PIN_TXD_UART_2, PIN_RXD_UART_2, PIN_RTS, PIN_CTS));

    return ESP_OK;
}

// Función para inicializar el bus I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) return ret;

    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// Función para convertir BCD a decimal
uint8_t bcd_to_decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Función para leer la hora del RTC DS1307
static esp_err_t ds1307_read_time(void) {
    uint8_t buffer[7];
    uint8_t reg = 0x00;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir en el RTC: %s", esp_err_to_name(ret));
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, sizeof(buffer) - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buffer + 6, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer del RTC: %s", esp_err_to_name(ret));
        return ret;
    }

    // Actualizar variables globales
    seconds = bcd_to_decimal(buffer[0] & 0x7F);
    minutes = bcd_to_decimal(buffer[1]);
    hours = bcd_to_decimal(buffer[2] & 0x3F);
    date = bcd_to_decimal(buffer[4]);
    month = bcd_to_decimal(buffer[5]);
    year = bcd_to_decimal(buffer[6]);

    ESP_LOGI(TAG, "Hora actual: %02d:%02d:%02d, Fecha: %02d/%02d/20%02d",
             hours, minutes, seconds, date, month, year);
    return ESP_OK;
}

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

esp_err_t adc_initialize() {
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

esp_err_t get_adc_value(adc_oneshot_unit_handle_t handle, adc_channel_t chan, int *out_raw) {
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
   // Compensar el desplazamiento inicial
    distances[sensor_index] -= 1.32;
    if (distances[sensor_index] < 0) {
        distances[sensor_index] = 0; // Evitar valores negativos
    }                            
}

void read_sensor(int sensor_index, adc_channel_t channel) {
    get_adc_value(adc1_handle, channel, &adc_raws[sensor_index]);
    adc_voltages[sensor_index] = adc_raws[sensor_index] * 3.3 / 4095.0;
    calculate_distance(sensor_index, adc_voltages[sensor_index]);
    ESP_LOGI(TAG, "Sensor %d Distance: %.2f mm", sensor_index + 1, distances[sensor_index]);
}

static void lora_task(void *arg) {
    char message[256];  // Incrementa el tamaño si es necesario para incluir todos los datos.

    // Configuración inicial para los comandos AT
    uart_write_bytes(UART_2_PORT, "AT+OPMODE=1\r\n", strlen("AT+OPMODE=1\r\n"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uart_write_bytes(UART_2_PORT, "AT+ADDRESS=200\r\n", strlen("AT+ADDRESS=200\r\n"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ds1307_read_time(); // Asegurarse de obtener la hora actual antes de enviar

    // Concatenar fecha y hora
    snprintf(message, sizeof(message), "Date:%02d/%02d/20%02d Time:%02d:%02d:%02d;",
             date, month, year, hours, minutes, seconds);

    // Concatenar datos de sensores habilitados
    for (int i = 0; i < N_SENSORS; i++) {
        if (sensor_states[i]) {
            char sensor_data[64];
            snprintf(sensor_data, sizeof(sensor_data), "S%d:%.2fmm;", i + 1, distances[i]);
            strncat(message, sensor_data, sizeof(message) - strlen(message) - 1);
        }
    }

    // Formato final del comando AT
    char at_command[300];
    snprintf(at_command, sizeof(at_command), "AT+SEND=200,%d,%s\r\n", strlen(message), message);
    uart_write_bytes(UART_2_PORT, at_command, strlen(at_command));

    ESP_LOGI(TAG, "Mensaje enviado por LoRa: %s", message);

    // Entrar en deep sleep después de enviar
    ESP_LOGI(TAG, "Entering deep sleep for 5 minutes...");
    esp_deep_sleep(43200000000);  // 12hrs en microsegundos
        // For 12 hours, change to: esp_deep_sleep(43200000000);
        // For 5 min, change to: esp_deep_sleep(300000000);
}

void app_main(void) {
    ESP_ERROR_CHECK(uart1_initialization());
    ESP_ERROR_CHECK(uart2_initialization());
    ESP_ERROR_CHECK(i2c_master_init()); // Inicialización del I2C para el RTC
    adc_initialize(); /* ADC initialization */
    gpio_initialize(); /* GPIO initialization */
    vTaskDelay(pdMS_TO_TICKS(500));
     
    while(1) {
        update_sensor_states(); /* Update sensor states from GPIO */

        if (sensor_states[0]) {
            adc_raws[1] = 0;
            adc_raws[2] = 0;
            read_sensor(0, ADC1_CHAN7);
        } else {
            ESP_LOGI(TAG, "Sensor 1 is disabled");
        }
       
        if (sensor_states[1]) {
            adc_raws[2] = 0;
            read_sensor(1, ADC1_CHAN4);
        } else {
            ESP_LOGI(TAG, "Sensor 2 is disabled");
        }
        
        if (sensor_states[2]) {
            read_sensor(2, ADC1_CHAN5);
        } else {
            ESP_LOGI(TAG, "Sensor 3 is disabled");
        }

        ds1307_read_time(); // Mostrar la hora del RTC
        vTaskDelay(pdMS_TO_TICKS(500));
        // Aquí se integra el proceso de inicialización de la SD
        ESP_LOGI(TAG, "Initializing SD card");
        const char *mount_point = "/sdcard"; // Punto de montaje
        sdmmc_card_t *card = NULL;          // Puntero para la tarjeta SD
        esp_err_t ret;

        // Configuración para el montaje
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = true,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        };

        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = PIN_NUM_MOSI,
            .miso_io_num = PIN_NUM_MISO,
            .sclk_io_num = PIN_NUM_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4000,
        };

        // Inicializar el bus SPI
        ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue; // Continuar con el siguiente ciclo
        }

        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = PIN_NUM_CS;
        slot_config.host_id = host.slot;

        // Montar el sistema de archivos
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to mount filesystem: %s", esp_err_to_name(ret));
            spi_bus_free(host.slot);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue; // Continuar con el siguiente ciclo
        }

        ESP_LOGI(TAG, "Filesystem mounted successfully");
        sdmmc_card_print_info(stdout, card);

        // Escribir un archivo en la tarjeta SD
        char data[256];
        snprintf(data, sizeof(data),
                "Date: %02d/%02d/20%02d, Time: %02d:%02d:%02d, Sensor 1: %.2f mm, Sensor 2: %.2f mm, Sensor 3: %.2f mm\n",
                date, month, year, hours, minutes, seconds, distances[0], distances[1], distances[2]);

        const char *file_path = "/sdcard/sensor.txt";

        FILE *file = fopen(file_path, "a");  
        if (file == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
        } else {
            // Escribir los datos en el archivo
            fprintf(file, "%s", data);
            fclose(file);
            ESP_LOGI(TAG, "Data saved successfully:\n%s", data);
        }

        // Asegurarse de que se desmonta correctamente el sistema de archivos después de escribir
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        ESP_LOGI(TAG, "Card unmounted");
        spi_bus_free(host.slot);


        // Desmontar el sistema de archivos
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        ESP_LOGI(TAG, "Card unmounted");
        spi_bus_free(host.slot);

        vTaskDelay(pdMS_TO_TICKS(5000));

        xTaskCreate(lora_task, "lora_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
