#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <string.h>

#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_TX_PIN 17 // Pin TX
#define UART_RX_PIN 16 // Pin RX
#define BUF_SIZE (1024)

#define BUTTON_GPIO_22 22 //pulsador
#define BUTTON_GPIO_23 23 //pulsador
QueueHandle_t cola;

static const char* TAG = "UART";

// Función para inicializar UART
void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}
// Tarea para detectar la pulsación del botón
void vTaskButton(void *pvParameters)
{
    uint8_t estado = 0;
    uint32_t elPin = (uint32_t)pvParameters;
    while (1)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        if (estado == 0 && gpio_get_level(elPin))
        {
            estado = 1;
            xQueueSend(cola, &elPin, 0);
            //ESP_LOGI(TAG, "Botón presionado. Enviado a la cola.");
        }
        if (estado == 1 && !gpio_get_level(elPin))
        {
            estado = 0;
        }
    }
}

// Tarea para enviar datos por UART cuando se detecta un evento
void vTaskSendUART(void *pvParameters) {
    uint32_t elPin;
    const char *msgAdelante = "adelante";
    const char *msgEnter = "enter";
    while (1) {
        // Verificamos si hay elementos en la cola
        uint8_t cantidad = uxQueueMessagesWaiting(cola);
        if (cantidad > 0) {
            if (xQueueReceive(cola, &elPin, 0) == pdPASS) {  // 0, No bloqueamos si la cola está vacía
                //ESP_LOGI(TAG, "Evento recibido: %d", elPin); // Verifica que el evento ha sido recibido correctamente
                if (elPin == BUTTON_GPIO_22) {
                    uart_write_bytes(UART_NUM, msgAdelante, strlen(msgAdelante));
                    //ESP_LOGI(TAG, "Mensaje enviado por uart: %s", msgAdelante); 
                }else if (elPin == BUTTON_GPIO_23)
                {
                    uart_write_bytes(UART_NUM, msgEnter, strlen(msgEnter));
                    //ESP_LOGI(TAG, "Mensaje enviado por uart: %s", msgEnter); 
                }
                
            }
        } else {
            //ESP_LOGI(TAG, "Cola vacía, esperando eventos...");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Espera para no saturar el CPU
    }
}


void app_main(void)
{
    uart_init();

    gpio_reset_pin(BUTTON_GPIO_22);
    gpio_set_direction(BUTTON_GPIO_22, GPIO_MODE_INPUT);

    gpio_reset_pin(BUTTON_GPIO_23);
    gpio_set_direction(BUTTON_GPIO_23, GPIO_MODE_INPUT);

    cola = xQueueCreate(20, sizeof(unsigned int));
    BaseType_t xReturned1, xReturned2;
    TaskHandle_t xHandle1, xHandle2 ;
    xReturned1 = xTaskCreate(vTaskButton, "DetectarPulsadorPin22", 4096, (void *)BUTTON_GPIO_22, 15, &xHandle1);
    xReturned2 = xTaskCreate(vTaskButton, "DetectarPulsadorPin23", 4096, (void *)BUTTON_GPIO_23, 15, &xHandle2);
    xTaskCreate(vTaskSendUART, "EnviarUART", 4096, NULL, 10, NULL);

    if (xReturned1 != pdPASS)
    {
        ESP_LOGI(TAG, "ERROR! no pude crear el task");
        while (1)
            ;
    }

    if (xReturned2 != pdPASS)
    {
        ESP_LOGI(TAG, "ERROR! no pude crear el task");
        while (1)
            ;
    }


    
}
