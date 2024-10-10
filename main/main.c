#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include <Fusion.h>

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

#define SAMPLE_PERIOD (0.01f) // Período de amostragem de 10 ms

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define DEAD_ZONE 10.0f // Zona morta aumentada para 10 graus
#define SCALE_FACTOR 255.0f / 360.0f // Mapear +/-360 graus para +/-255

QueueHandle_t xQueueData;

typedef struct {
    uint8_t type; // 0: movimento, 1: clique
    int16_t x;
    int16_t y;
} imu_data_t;

// Função para inicializar o MPU6050
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00}; // Acorda o MPU6050
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

// Função para ler dados brutos do MPU6050
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];

    // Começa a leitura a partir do registrador 0x3B (ACCEL_XOUT_H)
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true); // true para manter o controle do barramento
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    // Aceleração
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Temperatura
    *temp = (buffer[6] << 8) | buffer[7];

    // Giroscópio
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[(i + 4) * 2] << 8) | buffer[((i + 4) * 2) + 1];
    }
}

// Tarefa para ler dados do MPU6050, calcular orientação, detectar movimento e enviar dados via UART
void mpu6050_task(void *p) {
    // Inicializa o I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Inicializa o MPU6050
    mpu6050_reset();

    // Inicializa o Fusion AHRS
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3], temp;

    // Variáveis para detecção de clique
    float prev_accel_x = 0.0f;
    const float CLICK_THRESHOLD = 1.5f; // Threshold para detectar movimento repentino
    TickType_t last_click_time = 0;
    const TickType_t CLICK_DELAY = pdMS_TO_TICKS(500); // Atraso mínimo entre cliques

    // Variáveis estáticas para filtro passa-baixa
    static int16_t prev_x_movement = 0;
    static int16_t prev_y_movement = 0;
    const float SMOOTHING_FACTOR = 0.1f; // Fator de suavização (entre 0 e 1)

    while(1) {
        // Lê dados brutos do MPU6050
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Converte dados brutos em unidades físicas
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // G
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        // Atualiza o algoritmo AHRS
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Obtém os ângulos de Euler do algoritmo AHRS
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Mapeia roll e pitch para valores de movimento do mouse
        float roll = euler.angle.roll;
        float pitch = euler.angle.pitch;

        // Aplica zona morta
        if (fabsf(roll) < DEAD_ZONE) roll = 0.0f;
        if (fabsf(pitch) < DEAD_ZONE) pitch = 0.0f;

        // Mapeia ângulos para -255 a +255
        int16_t x_movement = (int16_t)(roll * SCALE_FACTOR);
        int16_t y_movement = (int16_t)(pitch * SCALE_FACTOR);

        // Limita valores a -255 e +255
        if (x_movement > 255) x_movement = 255;
        if (x_movement < -255) x_movement = -255;
        if (y_movement > 255) y_movement = 255;
        if (y_movement < -255) y_movement = -255;

        // Aplica fator de escala adicional
        x_movement /= 8;
        y_movement /= 8;

        // Aplica filtro passa-baixa
        x_movement = prev_x_movement + (int16_t)((x_movement - prev_x_movement) * SMOOTHING_FACTOR);
        y_movement = prev_y_movement + (int16_t)((y_movement - prev_y_movement) * SMOOTHING_FACTOR);

        // Atualiza os valores anteriores
        prev_x_movement = x_movement;
        prev_y_movement = y_movement;

        // Envia dados de movimento via UART
        imu_data_t data;
        data.type = 0; // Dados de movimento
        data.x = x_movement;
        data.y = y_movement;
        xQueueSend(xQueueData, &data, portMAX_DELAY);

        // Detecta movimento repentino para clique do mouse
        float accel_x = accelerometer.axis.x;

        if ((accel_x - prev_accel_x) > CLICK_THRESHOLD) {
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_click_time) > CLICK_DELAY) {
                // Envia evento de clique via UART
                imu_data_t click_event;
                click_event.type = 1; // Evento de clique
                click_event.x = 1;    // 1 para clique
                click_event.y = 0;
                xQueueSend(xQueueData, &click_event, portMAX_DELAY);
                last_click_time = current_time;
            }
        }

        prev_accel_x = accel_x;

        // Atraso para o período de amostragem
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Tarefa UART para enviar dados para o programa Python
void uart_task(void *p) {
    // Inicializa UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    imu_data_t data;

    while (1) {
        if (xQueueReceive(xQueueData, &data, portMAX_DELAY)) {
            if (data.type == 0) {
                // Dados de movimento
                uint8_t packet[7];
                packet[0] = 0xFF; // Byte de início
                packet[1] = 0x00; // Tipo: Movimento
                packet[2] = (data.x >> 8) & 0xFF;
                packet[3] = data.x & 0xFF;
                packet[4] = (data.y >> 8) & 0xFF;
                packet[5] = data.y & 0xFF;
                packet[6] = 0xFE; // Byte de fim
                uart_write_blocking(UART_ID, packet, sizeof(packet));
            } else if (data.type == 1) {
                // Evento de clique
                uint8_t packet[4];
                packet[0] = 0xFF; // Byte de início
                packet[1] = 0x01; // Tipo: Clique
                packet[2] = data.x & 0xFF; // 1 para clique
                packet[3] = 0xFE; // Byte de fim
                uart_write_blocking(UART_ID, packet, sizeof(packet));
            }
        }
    }
}

int main() {
    stdio_init_all();

    // Cria fila para comunicação entre tarefas
    xQueueData = xQueueCreate(10, sizeof(imu_data_t));

    // Cria tarefas
    xTaskCreate(mpu6050_task, "MPU6050 Task", 4096, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Task", 2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
