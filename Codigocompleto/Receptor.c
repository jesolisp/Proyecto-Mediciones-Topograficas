#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PIN_TXD_UART_1 1
#define PIN_RXD_UART_1 3
#define PIN_TXD_UART_2 17 /* Conectar al pin RX del módulo LoRa */
#define PIN_RXD_UART_2 16 /* Conectar al pin TX del módulo LoRa */
#define PIN_RTS UART_PIN_NO_CHANGE
#define PIN_CTS UART_PIN_NO_CHANGE

#define UART_1_PORT UART_NUM_1
#define UART_2_PORT UART_NUM_2
#define UART_BAUD_RATE  115200 /* Para comunicación con el PC */
#define UART_BAUD_RATE_LORA 9600 /* Según el datasheet, LoRa establece comunicación a 9600 baudios */
#define ECHO_TASK_STACK_SIZE 2048

static const char *TAG = "LoRa Config";

#define BUF_SIZE 1024

// Función para inicializar UART 1 (para comunicación con el PC)
esp_err_t uart1_initialization() {
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

// Función para inicializar UART 2 (para comunicación con LoRa)
esp_err_t uart2_initialization() {
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

// Función para recibir datos y mostrar el mensaje recibido
static void at_commands(void *arg) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Leer datos del UART (LoRa)
        int len = uart_read_bytes(UART_2_PORT, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // Null-terminate the received string
            ESP_LOGI(TAG, "Mensaje recibido: %s", (char *)data);
            
            // Escribir el mensaje recibido en el UART 1 (comunicación con PC)
            uart_write_bytes(UART_1_PORT, (const char *)data, len);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Espera corta antes de la próxima lectura
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(uart1_initialization());
    ESP_ERROR_CHECK(uart2_initialization());

    xTaskCreate(at_commands, "at_commands", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
