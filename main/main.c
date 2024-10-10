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

#define SAMPLE_PERIOD (0.01f) // Sample period in seconds

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

QueueHandle_t xQueueAdc;

typedef struct {
    int axis; // 0: X-axis, 1: Y-axis, 2: Click
    int val;
} adc_t;

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
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true); // Keep master control of bus
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

// Task to read MPU6050 data, compute orientation, and send data via queue
void mpu6050_task(void *p) {
    // Initialize I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Initialize MPU6050
    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;

    // Initialize Fusion AHRS
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    adc_t adc;

    // Variables for smoothing and sensitivity adjustments
    static float filtered_roll = 0.0f;
    static float filtered_pitch = 0.0f;
    const float alpha = 0.1f;       // Smoothing factor (adjust as needed)
    const float DEAD_ZONE = 10.0f;  // Degrees
    const float SCALE_FACTOR = 0.1f; // Adjust to reduce sensitivity

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

        // Invert roll axis if needed
        euler.angle.roll = -euler.angle.roll;

        // Apply dead zone
        if (fabsf(euler.angle.roll) < DEAD_ZONE) euler.angle.roll = 0.0f;
        if (fabsf(euler.angle.pitch) < DEAD_ZONE) euler.angle.pitch = 0.0f;

        // Apply smoothing filter
        filtered_roll = alpha * euler.angle.roll + (1.0f - alpha) * filtered_roll;
        filtered_pitch = alpha * euler.angle.pitch + (1.0f - alpha) * filtered_pitch;

        // Apply scaling factor to reduce sensitivity
        filtered_roll *= SCALE_FACTOR;
        filtered_pitch *= SCALE_FACTOR;

        // Clamp values to desired range
        filtered_roll = fminf(fmaxf(filtered_roll, -30.0f), 30.0f);
        filtered_pitch = fminf(fmaxf(filtered_pitch, -30.0f), 30.0f);

        // Send roll data (X-axis)
        adc.axis = 0; // X-axis
        adc.val = (int)filtered_roll;
        xQueueSend(xQueueAdc, &adc, portMAX_DELAY);

        // Delay between sending X and Y data
        vTaskDelay(pdMS_TO_TICKS(10));

        // Send pitch data (Y-axis)
        adc.axis = 1; // Y-axis
        adc.val = (int)filtered_pitch;
        xQueueSend(xQueueAdc, &adc, portMAX_DELAY);

        // Delay between sending data
        vTaskDelay(pdMS_TO_TICKS(10));

        // Detect click event based on acceleration in Z-axis
        if (accelerometer.axis.z > 1.5f) {
            adc.axis = 2; // Click event
            adc.val = 1;  // Click value (can be any non-zero value)
            xQueueSend(xQueueAdc, &adc, portMAX_DELAY);
        }

        // Delay before next reading
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// UART task to send data to the Python program
void uart_task(void *p) {
    adc_t data;

    while (1) {
        xQueueReceive(xQueueAdc, &data, portMAX_DELAY);

        // Send data via UART
        uint8_t axis = data.axis;
        putchar(axis);

        int16_t val = data.val;

        uint8_t val_1 = (val >> 8) & 0xFF;
        putchar(val_1);
        uint8_t val_0 = val & 0xFF;
        putchar(val_0);

        uint8_t eop = 0xFF; // End of packet
        putchar(eop);
    }
}

int main() {
    stdio_init_all();
    xQueueAdc = xQueueCreate(32, sizeof(adc_t));

    // Create tasks
    xTaskCreate(uart_task, "UART Task", 4096, NULL, 1, NULL);
    xTaskCreate(mpu6050_task, "MPU6050 Task", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
