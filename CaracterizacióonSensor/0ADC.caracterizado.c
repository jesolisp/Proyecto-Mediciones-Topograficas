/*
Autor: Mario García
Programa ESP32 ADC Para distancia con potenciometro lineal
date created: 25/06/24
last modified: 28/07/24
*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h" //Para hacer lecturas Oneshot o continuas
#include "esp_log.h"
#include "driver/uart.h" //Librería UART
#include "string.h"
//#include "esp_adc/adc_cali.h"  //Solo poner si se va a calibrar el ADC

//UART
#define UART_PORT_NUM UART_NUM_1 //Seleccionas el uart 1 que trae por default el ESP
#define TX_BUF_SIZE 1024 //Tamaño de buf de 8bits
#define TXD_PIN GPIO_NUM_1 //pin que manda 
#define RXD_PIN GPIO_NUM_3 //pin que recibe

#define ADC1_CHAN0 ADC_CHANNEL_4 //pin 32
#define ADC_ATTEN ADC_ATTEN_DB_11

adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw; 
static float voltage;
float distance_mm;

esp_err_t config_ADC();
esp_err_t get_ADC_value();

///////////////////////////UART
esp_err_t uart_initialize();
static void tx_task(void *arg);

void app_main(){
    config_ADC(); 
    uart_initialize();
    //creamos tarea tx_task, lleva 
    xTaskCreate(tx_task, "uart_tx_task", TX_BUF_SIZE*2, NULL, configMAX_PRIORITIES-1, NULL); //El buf es una pila de datos, si se llena se tiene que vaciar para recibir datos
}

static void tx_task(void *arg){
    char str[80];
    
    while (1) {

        get_ADC_value();
        vTaskDelay(250/ portTICK_PERIOD_MS);
        const int len = strlen(str); //cuenta tamaño de la cadena //len tiene el tamaño de la cadena
        uart_write_bytes(UART_PORT_NUM, str, len+1); //longitud de la cadena, se puede poner len-1, etc para cortar un caracter
        sprintf(str,"/*%2.2f, %2.2f */", voltage, distance_mm);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

esp_err_t config_ADC() {
    //ADC init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    //ADC config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, //ancho de banda
        .atten = ADC_ATTEN, //Atenuacion
    };

    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);
    
    return ESP_OK; 
}

esp_err_t get_ADC_value(){

    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    // printf("Raw data: %d\n", adc_raw);

    // Recordar que el ADC es de 12 bits (4096) 
    voltage = (adc_raw * 3.3 / 4095.0); // importante poner el cero, para forzar la interpretación de la división como una operación de punto flotante.
    // printf("Voltage: %2.2f V\n", voltage);

    // Convertir el voltaje a distancia en milímetros usando la ecuación polinómica
    float p1 = 0.43845;
    float p2 = -3.1136;
    float p3 = 6.6372;
    float p4 = -5.4496;
    float p5 = 35.144;
    float p6 = 0.53767;
    
    distance_mm = p1*voltage*voltage*voltage*voltage*voltage + p2*voltage*voltage*voltage*voltage + p3*voltage*voltage*voltage + p4*voltage*voltage + p5*voltage + p6;
    // printf("Distance: %2.2f mm\n", distance_mm);

    return ESP_OK;

}

esp_err_t uart_initialize(){  
    const uart_config_t uart_config = {
        .baud_rate = 115200,  //baudios
        .data_bits = UART_DATA_8_BITS,  //1byte
        .parity = UART_PARITY_DISABLE, //Paritybit, checa si se envian correctamente los datos por bit, al final cuenta los datos que se enviaron correctamente, par correcto, impar incorrecto
        .stop_bits = UART_STOP_BITS_1, //bit de paro
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, TX_BUF_SIZE * 2, 0, 0, NULL, 0)); //2 datos, uno null y otro dato
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}
