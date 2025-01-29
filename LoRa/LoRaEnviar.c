#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

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

static const char *TAG = "LoRa Config";

#define BUF_SIZE 1024

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

static void lora_task(void *arg){
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    // Configuración inicial para los comandos AT
    uart_write_bytes(UART_2_PORT, "AT+OPMODE=1\r\n", strlen("AT+OPMODE=1\r\n"));  // Modo propietario
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Espera para asegurar que se aplica el comando

    uart_write_bytes(UART_2_PORT, "AT+ADDRESS=200\r\n", strlen("AT+ADDRESS=200\r\n"));  // Dirección del LoRa
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Espera para que se configure

    while (1) {
        // Enviar "HELLO" al otro LoRa
        uart_write_bytes(UART_2_PORT, "AT+SEND=200,5,HELLO\r\n", strlen("AT+SEND=200,5,HELLO\r\n"));

        ESP_LOGI(TAG, "Sent 'HELLO' to LoRa module.");

        // Espera de 5 segundos
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        
        // Recibir respuesta (opcional)
        int len = uart_read_bytes(UART_2_PORT, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Received: %s", data);
        }
    }
}

void app_main(void){
    ESP_ERROR_CHECK(uart1_initialization());
    ESP_ERROR_CHECK(uart2_initialization());

    xTaskCreate(lora_task, "lora_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
