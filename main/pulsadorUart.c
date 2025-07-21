#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <string.h>
#include <math.h>
#include "esp_check.h"
#include "driver/dac_continuous.h"
#include "driver/dac_cosine.h"

#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_TX_PIN 17 // Pin TX
#define UART_RX_PIN 16 // Pin RX
#define BUF_SIZE (1024)

#define BUTTON_GPIO_22 22 // pulsador
#define BUTTON_GPIO_23 23 // pulsador

// #define 2_PI 6.2832 // 2 * PI
#define TAM_ARRAY 400

static const char *TAG = "UART";

typedef struct
{
    uint8_t type;
    uint8_t ampl;
    uint16_t freq;
    uint16_t phase;
} signal_config;

signal_config signal;
uint8_t wave[TAM_ARRAY];

QueueHandle_t cola;

// Función para inicializar UART
void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
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
            // ESP_LOGI(TAG, "Botón presionado. Enviado a la cola.");
        }
        if (estado == 1 && !gpio_get_level(elPin))
        {
            estado = 0;
        }
    }
}

// Tarea para enviar datos por UART cuando se detecta un evento
void uart_send_task(void *pvParameters)
{
    uint32_t elPin;
    const char *msgAdelante = "adelante";
    const char *msgEnter = "enter";
    while (1)
    {
        // Verificamos si hay elementos en la cola
        uint8_t cantidad = uxQueueMessagesWaiting(cola);
        if (cantidad > 0)
        {
            if (xQueueReceive(cola, &elPin, 0) == pdPASS)
            { // 0, No bloqueamos si la cola está vacía
                // ESP_LOGI(TAG, "Evento recibido: %d", elPin); // Verifica que el evento ha sido recibido correctamente
                if (elPin == BUTTON_GPIO_22)
                {
                    uart_write_bytes(UART_NUM, msgAdelante, strlen(msgAdelante));
                    // ESP_LOGI(TAG, "Mensaje enviado por uart: %s", msgAdelante);
                }
                else if (elPin == BUTTON_GPIO_23)
                {
                    uart_write_bytes(UART_NUM, msgEnter, strlen(msgEnter));
                    // ESP_LOGI(TAG, "Mensaje enviado por uart: %s", msgEnter);
                }
            }
        }
        else
        {
            // ESP_LOGI(TAG, "Cola vacía, esperando eventos...");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Espera para no saturar el CPU
    }
}

dac_continuous_handle_t dac_continuous_handle;
dac_continuous_config_t dac_config = {
    .chan_mask = DAC_CHANNEL_MASK_ALL,
    .desc_num = 8,
    .buf_size = TAM_ARRAY, // 2048
    .freq_hz = TAM_ARRAY * 2000,
    .offset = 0,
    .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
    .chan_mode = DAC_CHANNEL_MODE_SIMUL,
};

// genera muestras digitales para cuatro tipos de ondas
void generador_de_funciones(uint8_t signal_type, uint8_t amplitude_volts, uint16_t phase)
{
    uint32_t tam = TAM_ARRAY;
    uint8_t amplitude = (uint8_t)((amplitude_volts / 3.3) * 255); // Convierte la amplitud a un valor de 8 bits
    for (int i = 0; i < tam; i++)
    {
        switch (signal_type)
        {
        case 0: // Onda Senoidal
            wave[i] = (uint8_t)((sin(i * 2 * M_PI / tam + (phase / sizeof(uint16_t)) * 2 * M_PI / 360) + 1) * (double)(amplitude) / 2 + 0.5);
            break;
        case 1: // Onda Cuadrada
            wave[i] = (i < tam / 2) ? amplitude : 0;
            break;
        case 2: // Onda Diente de Sierra
            wave[i] = (i * amplitude) / tam;
            break;
        case 3: // Onda Triangular
            wave[i] = (i < tam / 2) ? (2 * amplitude * i / tam) : (2 * amplitude * (tam - i) / tam);
            break;
        default: // Onda Senoidal por defecto
            wave[i] = (uint8_t)((sin(2 * M_PI * i / tam) + 1) * amplitude / 2);
            break;
        }
    }
}

void update_dac_config()
{
    generador_de_funciones(signal.type, signal.ampl, signal.phase);
    dac_config.freq_hz = signal.freq * TAM_ARRAY;

    // Reinicializar DAC con nueva configuración
    if (dac_continuous_disable(dac_continuous_handle) != ESP_OK)
        ESP_LOGW(TAG, "No se pudo deshabilitar el DAC");

    if (dac_continuous_del_channels(dac_continuous_handle) != ESP_OK)
        ESP_LOGW(TAG, "No se pudo liberar el canal DAC");

    esp_err_t status = dac_continuous_new_channels(&dac_config, &dac_continuous_handle);
    if (status == ESP_OK)
    {
        status = dac_continuous_enable(dac_continuous_handle);
        if (status != ESP_OK)
        {
            ESP_LOGE(TAG, "Fallo al habilitar el DAC: %s", esp_err_to_name(status));
        }
    }
    else
    {
        ESP_LOGE(TAG, "Fallo al crear el canal DAC: %s", esp_err_to_name(status));
    }
}

static void uart_receive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Recibiendo UART");
    // uint8_t rx_uart[BUF_SIZE] = {0};
    char rx_uart[BUF_SIZE] = {0};
    // size_t tamano_buffer = TAM_ARRAY;

    while (1)
    {
        int bytes_read = uart_read_bytes(UART_NUM, rx_uart, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (bytes_read > 0)
        {
            rx_uart[bytes_read] = '\0'; // Asegurar que sea una cadena válida

            ESP_LOGI(TAG, "Mensaje recibido: %s", rx_uart);

            int freq = 0;
            int amp = 0;
            int signal_type = 0;

            char *token = strtok(rx_uart, ",");
            // Extraer el primer dato (frecuencia)
            if (token != NULL)
            {
                freq = atoi(token); // Convierte una cadena a entero
                ESP_LOGI(TAG, "Frecuencia: %d", freq);
            } 
            
            // Extraer el segundo dato (amplitud)
            token = strtok(NULL, ","); // Obtiene el siguiente token después de la coma
            if (token != NULL)
            {
                amp = atoi(token);
                ESP_LOGI(TAG, "Amplitud: %d", amp);
            }

            token = strtok(NULL, ","); 
            if (token != NULL)
            {
                signal_type = atoi(token);
                ESP_LOGI(TAG, "Tipo de señal: %d", signal_type);
            }

            signal.type = signal_type;
            signal.freq = freq;
            signal.ampl = amp;
            signal.phase = 0;

            update_dac_config();
        }

        dac_continuous_write_cyclically(dac_continuous_handle, (uint8_t *)wave, TAM_ARRAY, NULL);

        vTaskDelay(100 / portTICK_PERIOD_MS);
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
    TaskHandle_t xHandle1, xHandle2;
    xReturned1 = xTaskCreate(vTaskButton, "DetectarPulsadorPin22", 4096, (void *)BUTTON_GPIO_22, 15, &xHandle1);
    xReturned2 = xTaskCreate(vTaskButton, "DetectarPulsadorPin23", 4096, (void *)BUTTON_GPIO_23, 15, &xHandle2);
    xTaskCreate(uart_send_task, "EnviarUART", 4096, NULL, 10, NULL);

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

    // Genera una señal inicial antes de usar la interfaz grafica
    generador_de_funciones(1, 1, 0);
    dac_continuous_new_channels(&dac_config, &dac_continuous_handle);
    dac_continuous_enable(dac_continuous_handle);

    xTaskCreate(uart_receive_task, "UartReceiveTask", 4096, NULL, 5, NULL);
}
