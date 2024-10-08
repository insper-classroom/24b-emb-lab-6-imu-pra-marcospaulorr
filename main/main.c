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

#define SAMPLE_PERIOD (0.01f) // 10 ms sample period

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define DEAD_ZONE 5.0f // Degrees
#define SCALE_FACTOR 128.0f / 45.0f // Map +/-45 degrees to +/-128

QueueHandle_t xQueueData;

typedef struct {
    uint8_t type; // 0: movement, 1: click
    int16_t x;
    int16_t y;
} imu_data_t;

// Function to initialize the MPU6050
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00}; // Wake up the MPU6050
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

// Function to read raw data from MPU6050
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];

    // Start reading from register 0x3B (ACCEL_XOUT_H)
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    // Acceleration
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Temperature
    *temp = (buffer[6] << 8) | buffer[7];

    // Gyroscope
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[(i + 4) * 2] << 8) | buffer[((i + 4) * 2) + 1];
    }
}

// Task to read MPU6050 data, compute orientation, detect movement, and send data via UART
void mpu6050_task(void *p) {
    // Initialize I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Initialize MPU6050
    mpu6050_reset();

    // Initialize Fusion AHRS
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3], temp;

    // Variables for click detection
    float prev_accel_x = 0.0f;
    const float CLICK_THRESHOLD = 1.5f; // Threshold for detecting sudden movement
    TickType_t last_click_time = 0;
    const TickType_t CLICK_DELAY = pdMS_TO_TICKS(500); // Minimum delay between clicks

    while(1) {
        // Read raw data from MPU6050
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Convert raw data to physical units
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Degrees/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // G
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        // Update the AHRS algorithm
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Get Euler angles from the AHRS algorithm
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Map roll and pitch to mouse movement values
        float roll = euler.angle.roll;
        float pitch = euler.angle.pitch;

        // Apply dead zone
        if (fabsf(roll) < DEAD_ZONE) roll = 0.0f;
        if (fabsf(pitch) < DEAD_ZONE) pitch = 0.0f;

        // Map angles to -128 to +128
        int16_t x_movement = (int16_t)(roll * SCALE_FACTOR);
        int16_t y_movement = (int16_t)(pitch * SCALE_FACTOR);

        // Clamp values to -128 to +128
        if (x_movement > 128) x_movement = 128;
        if (x_movement < -128) x_movement = -128;
        if (y_movement > 128) y_movement = 128;
        if (y_movement < -128) y_movement = -128;

        // Optionally, apply an additional scaling factor
        x_movement /= 2;
        y_movement /= 2;

        // Send movement data via UART
        imu_data_t data;
        data.type = 0; // Movement data
        data.x = x_movement;
        data.y = y_movement;
        xQueueSend(xQueueData, &data, portMAX_DELAY);

        // Detect sudden movement for mouse click
        float accel_x = accelerometer.axis.x;

        if ((accel_x - prev_accel_x) > CLICK_THRESHOLD) {
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_click_time) > CLICK_DELAY) {
                // Send click event via UART
                imu_data_t click_event;
                click_event.type = 1; // Click event
                click_event.x = 1;    // 1 for click
                click_event.y = 0;
                xQueueSend(xQueueData, &click_event, portMAX_DELAY);
                last_click_time = current_time;
            }
        }

        prev_accel_x = accel_x;

        // Delay for the sample period
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// UART task to send data to the Python program
void uart_task(void *p) {
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    imu_data_t data;

    while (1) {
        if (xQueueReceive(xQueueData, &data, portMAX_DELAY)) {
            if (data.type == 0) {
                // Movement data
                uint8_t packet[7];
                packet[0] = 0xFF; // Start byte
                packet[1] = 0x00; // Type: Movement
                packet[2] = (data.x >> 8) & 0xFF;
                packet[3] = data.x & 0xFF;
                packet[4] = (data.y >> 8) & 0xFF;
                packet[5] = data.y & 0xFF;
                packet[6] = 0xFE; // End byte
                uart_write_blocking(UART_ID, packet, sizeof(packet));
            } else if (data.type == 1) {
                // Click event
                uint8_t packet[4];
                packet[0] = 0xFF; // Start byte
                packet[1] = 0x01; // Type: Click
                packet[2] = data.x & 0xFF; // 1 for click
                packet[3] = 0xFE; // End byte
                uart_write_blocking(UART_ID, packet, sizeof(packet));
            }
        }
    }
}

int main() {
    stdio_init_all();

    // Create queue for communication between tasks
    xQueueData = xQueueCreate(10, sizeof(imu_data_t));

    // Create tasks
    xTaskCreate(mpu6050_task, "MPU6050 Task", 4096, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Task", 2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
