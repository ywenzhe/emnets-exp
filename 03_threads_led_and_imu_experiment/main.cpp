#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <string>
#include <log.h>
#include <errno.h>
#include "clk.h"
#include "board.h"
#include "periph_conf.h"
#include "timex.h"
#include "ztimer.h"
#include "periph/gpio.h"
#include "thread.h"
#include "msg.h"
#include "shell.h"
// #include "xtimer.h"
#include "ledcontroller.hh"
#include "mpu6050.h"
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack_for_led_thread[THREAD_STACKSIZE];
static char stack_for_imu_thread[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
#define LED_MSG_TYPE_ISR     (0x3456)
#define LED_GPIO_R GPIO26
#define LED_GPIO_G GPIO25
#define LED_GPIO_B GPIO27

#define MSG_QUEUE_SIZE 8
static msg_t _msg_queue[MSG_QUEUE_SIZE];
struct MPU6050Data {
    float ax, ay, az;
    float gx, gy, gz;
};
enum MoveState { Stationary, Tilted, Rotating, Moving };

void delay_ms(uint32_t sleep_ms) {
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}
/**
 * LED control thread function.
 * Then, it enters an infinite loop where it waits for messages to control the LED.
 * @param arg Unused argument.
 * @return NULL.
 */
void* _led_thread(void* arg) {
    (void)arg;
    LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
    led.change_led_color(COLOR_NONE);

    msg_init_queue(_msg_queue, MSG_QUEUE_SIZE);

    msg_t msg;

    while (1) {
        // Wait for a message to control the LED
        msg_receive(&msg);

        // Display different light colors based on the motion state of the device
        switch (msg.type) {
        case Stationary:
            led.change_led_color(COLOR_NONE);     // Off for horizontal stationary
            break;
        case Tilted:
            led.change_led_color(COLOR_RED);      // Red for tilted stationary
            break;
        case Rotating:
            led.change_led_color(COLOR_BLUE);     // Blue for rotating
            break;
        case Moving:
            led.change_led_color(COLOR_GREEN);    // Green for translation (moving)
            break;
        default:
            led.change_led_color(COLOR_NONE);     // Off for unknown state
            break;
        }
    }
    return NULL;
}

#define g_acc (9.8)
MoveState detectMovement(MPU6050Data& data) {
    // Calculate magnitudes
    float accel_magnitude = sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    float gyro_magnitude = sqrt(data.gx * data.gx + data.gy * data.gy + data.gz * data.gz);

    // Threshold values for motion detection
    const float ACCEL_STATIONARY_THRESHOLD = 1.2f;  // Acceleration threshold for stationary detection
    const float ACCEL_MOVING_THRESHOLD = 2.5f;      // Acceleration threshold for moving detection
    const float GYRO_ROTATING_THRESHOLD = 30.0f;    // Gyroscope threshold for rotating (degrees/second)
    const float TILT_THRESHOLD = 0.2f;              // Threshold for tilt detection (20% of gravity)
    const float HORIZONTAL_THRESHOLD = 0.1f;        // Threshold for horizontal detection

    // First, check if device is stationary (close to 1g acceleration and minimal rotation)
    if (fabs(accel_magnitude - g_acc) < ACCEL_STATIONARY_THRESHOLD && gyro_magnitude < 15.0f) {
        // Device is stationary, now check if it's horizontal or tilted

        // For horizontal stationary: acceleration should be mostly in Z direction
        // Z should be close to 9.8 m/s², X and Y should be close to 0
        if (fabs(data.ax) < HORIZONTAL_THRESHOLD * g_acc &&
            fabs(data.ay) < HORIZONTAL_THRESHOLD * g_acc &&
            fabs(data.az - g_acc) < HORIZONTAL_THRESHOLD * g_acc) {
            return Stationary;  // Horizontal stationary
        }

        // For tilted stationary: significant horizontal acceleration component
        float horizontal_accel = sqrt(data.ax * data.ax + data.ay * data.ay);
        if (horizontal_accel > TILT_THRESHOLD * g_acc) {
            return Tilted;     // Tilted stationary
        }

        // Default to stationary if acceleration is close to gravity
        return Stationary;
    }

    // Check for rotation (high gyroscope values with moderate acceleration)
    if (gyro_magnitude > GYRO_ROTATING_THRESHOLD) {
        return Rotating;
    }

    // Check for translation/movement (high acceleration changes but not just due to orientation)
    if (fabs(accel_magnitude - g_acc) > ACCEL_MOVING_THRESHOLD) {
        return Moving;
    }

    // Default to stationary if no clear motion detected
    return Stationary;
}

void* _imu_thread(void* arg) {
    (void)arg;

    // 1. Initialize MPU6050 sensor
    MPU6050 mpu(MPU6050_IIC_DEV, MPU6050_DEFAULT_ADDRESS);

    printf("[IMU] Initializing MPU6050 sensor...\n");
    mpu.initialize();

    if (!mpu.testConnection()) {
        printf("[IMU] ERROR: MPU6050 connection failed!\n");
        return NULL;
    }

    printf("[IMU] MPU6050 initialized successfully\n");

    // Set sensor configuration for optimal performance
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g range for precision
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250°/s range
    mpu.setRate(100);  // 100Hz sampling rate
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);  // Digital Low Pass Filter for noise reduction

    MPU6050Data sensorData;
    MoveState currentState, previousState = Stationary;
    msg_t msg;

    printf("[IMU] Starting sensor data acquisition...\n");

    while (1) {
        // 2. Acquire sensor data every 100ms
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // Convert raw data to physical units
        sensorData.ax = ax_raw / 16384.0f * g_acc;  // Convert to m/s² (±2g range)
        sensorData.ay = ay_raw / 16384.0f * g_acc;
        sensorData.az = az_raw / 16384.0f * g_acc;
        sensorData.gx = gx_raw / 131.0f;  // Convert to °/s (±250°/s range)
        sensorData.gy = gy_raw / 131.0f;
        sensorData.gz = gz_raw / 131.0f;

        // 3. Determine the motion state
        currentState = detectMovement(sensorData);

        // 4. Notify the LED thread to display the light color through a message
        // Send message only when state changes to avoid flooding
        if (currentState != previousState) {
            msg.type = (uint32_t)currentState;

            if (msg_send(&msg, _led_pid) == 0) {
                const char* state_names[] = { "Stationary", "Tilted", "Rotating", "Moving" };
                printf("[IMU] State changed: %s -> %s\n",
                    state_names[previousState], state_names[currentState]);
            } else {
                printf("[IMU] Failed to send message to LED thread\n");
            }

            previousState = currentState;
        }

        // Print sensor data for debugging (every 5 seconds)
        static int counter = 0;
        if (counter++ % 50 == 0) {
            const char* state_names[] = { "Stationary", "Tilted", "Rotating", "Moving" };
            printf("[IMU] Accel: (%.2f, %.2f, %.2f) m/s², Gyro: (%.1f, %.1f, %.1f) °/s, State: %s\n",
                sensorData.ax, sensorData.ay, sensorData.az,
                sensorData.gx, sensorData.gy, sensorData.gz, state_names[currentState]);
        }

        // Wait 100ms before next reading
        delay_ms(100);
    }

    return NULL;
}
static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

int main(void) {
    _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread), THREAD_PRIORITY_MAIN - 2,
        THREAD_CREATE_STACKTEST, _led_thread, NULL,
        "led_controller_thread");
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    thread_create(stack_for_imu_thread, sizeof(stack_for_imu_thread), THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST, _imu_thread, NULL,
        "imu_read_thread");
    printf("[Main] Initialization successful - starting the shell now\n");
    while (1) {

    }
    return 0;
}
