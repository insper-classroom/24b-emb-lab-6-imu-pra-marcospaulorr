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

#define SAMPLE_INTERVAL (0.01f) // Intervalo de amostragem em segundos

#define MPU6050_ADDRESS 0x68
#define SDA_PIN 4
#define SCL_PIN 5

#define DEAD_ZONE_ANGLE 10.0f   // Zona morta em graus
#define SENSITIVITY_FACTOR 0.1f // Fator para ajustar a sensibilidade

QueueHandle_t dataQueue;

typedef struct {
    uint8_t axis; // 0: X, 1: Y, 2: Clique
    int16_t value;
} imu_data_t;

// Função para inicializar o sensor MPU6050
static void initialize_mpu6050() {
    uint8_t config[] = {0x6B, 0x00}; // Wake up the MPU6050
    i2c_write_blocking(i2c_default, MPU6050_ADDRESS, config, 2, false);
}

// Função para ler dados brutos do MPU6050
static void read_mpu6050_raw(int16_t accel_data[3], int16_t gyro_data[3], int16_t *temperature) {
    uint8_t buffer[14];
    uint8_t reg = 0x3B;

    // Envia o registrador inicial para leitura
    i2c_write_blocking(i2c_default, MPU6050_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDRESS, buffer, 14, false);

    // Processa dados de aceleração
    for (int i = 0; i < 3; i++) {
        accel_data[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }

    // Processa temperatura
    *temperature = (buffer[6] << 8) | buffer[7];

    // Processa dados do giroscópio
    for (int i = 0; i < 3; i++) {
        gyro_data[i] = (buffer[(i + 4) * 2] << 8) | buffer[(i + 4) * 2 + 1];
    }
}

// Tarefa para ler dados do MPU6050 e enviar para a fila
void imu_task(void *params) {
    // Inicializa I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Inicializa MPU6050
    initialize_mpu6050();

    int16_t accel[3], gyro[3], temp;
    FusionAhrs fusionAhrs;
    FusionAhrsInitialise(&fusionAhrs);

    imu_data_t imuData;

    // Variáveis para suavização e ajustes
    float smoothed_roll = 0.0f;
    float smoothed_pitch = 0.0f;
    const float smoothing_alpha = 0.1f; // Fator de suavização

    while (1) {
        // Lê dados brutos do sensor
        read_mpu6050_raw(accel, gyro, &temp);

        // Converte dados para unidades físicas
        FusionVector gyro_vector = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accel_vector = {
            .axis.x = accel[0] / 16384.0f,
            .axis.y = accel[1] / 16384.0f,
            .axis.z = accel[2] / 16384.0f,
        };

        // Atualiza o AHRS
        FusionAhrsUpdateNoMagnetometer(&fusionAhrs, gyro_vector, accel_vector, SAMPLE_INTERVAL);

        // Obtém ângulos de Euler
        FusionEuler eulerAngles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusionAhrs));

        // Inverte o eixo de rotação se necessário
        eulerAngles.angle.roll = -eulerAngles.angle.roll;

        // Aplica zona morta
        if (fabsf(eulerAngles.angle.roll) < DEAD_ZONE_ANGLE) eulerAngles.angle.roll = 0.0f;
        if (fabsf(eulerAngles.angle.pitch) < DEAD_ZONE_ANGLE) eulerAngles.angle.pitch = 0.0f;

        // Aplica filtro de suavização
        smoothed_roll = smoothing_alpha * eulerAngles.angle.roll + (1.0f - smoothing_alpha) * smoothed_roll;
        smoothed_pitch = smoothing_alpha * eulerAngles.angle.pitch + (1.0f - smoothing_alpha) * smoothed_pitch;

        // Ajusta sensibilidade
        smoothed_roll *= SENSITIVITY_FACTOR;
        smoothed_pitch *= SENSITIVITY_FACTOR;

        // Limita os valores ao intervalo desejado
        smoothed_roll = fminf(fmaxf(smoothed_roll, -30.0f), 30.0f);
        smoothed_pitch = fminf(fmaxf(smoothed_pitch, -30.0f), 30.0f);

        // Envia dados do eixo X
        imuData.axis = 0; // X-axis
        imuData.value = (int16_t)smoothed_roll;
        xQueueSend(dataQueue, &imuData, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));

        // Envia dados do eixo Y
        imuData.axis = 1; // Y-axis
        imuData.value = (int16_t)smoothed_pitch;
        xQueueSend(dataQueue, &imuData, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));

        // Detecta clique baseado na aceleração no eixo Z
        if (accel_vector.axis.z > 1.5f) {
            imuData.axis = 2; // Clique
            imuData.value = 1; // Valor do clique
            xQueueSend(dataQueue, &imuData, portMAX_DELAY);
        }

        // Pequeno atraso antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Tarefa UART para enviar dados para o programa Python
void uart_sender_task(void *params) {
    imu_data_t receivedData;

    while (1) {
        // Recebe dados da fila
        xQueueReceive(dataQueue, &receivedData, portMAX_DELAY);

        // Envia dados via UART
        uint8_t axis = receivedData.axis;
        putchar(axis);

        int16_t value = receivedData.value;

        uint8_t high_byte = (value >> 8) & 0xFF;
        putchar(high_byte);
        uint8_t low_byte = value & 0xFF;
        putchar(low_byte);

        uint8_t end_marker = 0xFF; // Marcador de fim de pacote
        putchar(end_marker);
    }
}

int main() {
    stdio_init_all();
    dataQueue = xQueueCreate(32, sizeof(imu_data_t));

    // Cria as tarefas
    xTaskCreate(uart_sender_task, "UART Sender Task", 4096, NULL, 1, NULL);
    xTaskCreate(imu_task, "IMU Task", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1)
        ;
}
