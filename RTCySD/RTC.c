#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

// Definición de etiquetas y pines I2C
#define I2C_MASTER_SCL_IO           22      // Pin para SCL
#define I2C_MASTER_SDA_IO           21      // Pin para SDA
#define I2C_MASTER_FREQ_HZ          100000  // Frecuencia del bus I2C
#define I2C_MASTER_PORT             I2C_NUM_0
#define DS1307_ADDR                 0x68    // Dirección del RTC DS1307

static const char *TAG = "DS1307";

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
static uint8_t bcd_to_decimal(uint8_t val) {
    return ((val / 16 * 10) + (val % 16));
}

// Función para convertir decimal a BCD
static uint8_t decimal_to_bcd(uint8_t val) {
    return ((val / 10 * 16) + (val % 10));
}

// Función para leer los datos del DS1307
static esp_err_t ds1307_read_time(void) {
    uint8_t buffer[7];
    uint8_t reg = 0x00;

    // Escribir dirección de registro inicial
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

    // Leer datos del RTC
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

    // Convertir y mostrar la hora
    uint8_t seconds = bcd_to_decimal(buffer[0] & 0x7F);
    uint8_t minutes = bcd_to_decimal(buffer[1]);
    uint8_t hours = bcd_to_decimal(buffer[2] & 0x3F); // Formato 24 horas
    uint8_t day = bcd_to_decimal(buffer[3]);
    uint8_t date = bcd_to_decimal(buffer[4]);
    uint8_t month = bcd_to_decimal(buffer[5]);
    uint8_t year = bcd_to_decimal(buffer[6]);

    ESP_LOGI(TAG, "Hora actual: %02d:%02d:%02d, Fecha: %02d/%02d/20%02d",
             hours, minutes, seconds, date, month, year);
    return ESP_OK;
}

// Función para establecer la hora inicial en el DS1307
static esp_err_t ds1307_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds,
                                 uint8_t day, uint8_t date, uint8_t month, uint8_t year) {
    uint8_t buffer[7];
    buffer[0] = decimal_to_bcd(seconds & 0x7F);  // Clear CH bit (bit 7) to start the clock
    buffer[1] = decimal_to_bcd(minutes);
    buffer[2] = decimal_to_bcd(hours & 0x3F);    // 24-hour format
    buffer[3] = decimal_to_bcd(day);             // Day of the week (1-7)
    buffer[4] = decimal_to_bcd(date);            // Date (1-31)
    buffer[5] = decimal_to_bcd(month);           // Month (1-12)
    buffer[6] = decimal_to_bcd(year);            // Year (0-99)

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Dirección del registro de inicio
    i2c_master_write(cmd, buffer, sizeof(buffer), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Hora inicial configurada correctamente");
    } else {
        ESP_LOGE(TAG, "Error al configurar la hora inicial: %s", esp_err_to_name(ret));
    }
    return ret;
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado correctamente");

    // Configura una hora inicial si el reloj no está corriendo
    ESP_ERROR_CHECK(ds1307_set_time(12, 0, 0, 2, 1, 1, 23)); // 12:00:00, lunes, 1/1/2023

    while (1) {
        ESP_ERROR_CHECK(ds1307_read_time());
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Leer cada segundo
    }
}
