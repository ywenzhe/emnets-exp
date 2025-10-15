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

struct MPU6050Data {
    float ax, ay, az;
    float gx, gy, gz;
};
enum MoveState {
    Stationary,        // 水平静止（不亮灯）
    X_Axis_Moving,     // X轴方向平移（黄灯）
    Y_Axis_Moving,     // Y轴方向平移（绿灯）
    Z_Axis_Moving,     // Z轴方向平移（青灯）
    X_Axis_Rotating,   // 绕X轴旋转（蓝灯）
    Y_Axis_Rotating,   // 绕Y轴旋转（洋红灯）
    Z_Axis_Rotating,   // 绕Z轴旋转（白灯）
    Tilted_Stationary  // 倾斜静止（红灯）
};

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

    msg_t msg;

    while (1) {
        // Wait for a message to control the LED
        msg_receive(&msg);

        // Display different light colors based on the motion state of the device
        switch (msg.type) {
        case Stationary:
            led.change_led_color(COLOR_NONE);     // Off for horizontal stationary
            break;
        case X_Axis_Moving:
            led.change_led_color(COLOR_YELLO);    // Yellow for X-axis translation
            break;
        case Y_Axis_Moving:
            led.change_led_color(COLOR_GREEN);    // Green for Y-axis translation
            break;
        case Z_Axis_Moving:
            led.change_led_color(COLOR_CYAN);     // Cyan for Z-axis translation
            break;
        case X_Axis_Rotating:
            led.change_led_color(COLOR_BLUE);     // Blue for X-axis rotation
            break;
        case Y_Axis_Rotating:
            led.change_led_color(COLOR_MAGENTA);  // Magenta for Y-axis rotation
            break;
        case Z_Axis_Rotating:
            led.change_led_color(COLOR_WHITE);    // White for Z-axis rotation
            break;
        case Tilted_Stationary:
            led.change_led_color(COLOR_RED);      // Red for tilted stationary
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
    const float ACCEL_STATIONARY_THRESHOLD = 1.0f;  // Acceleration threshold for stationary detection
    const float ACCEL_MOVING_THRESHOLD = 2.0f;      // Acceleration threshold for axis-specific movement
    const float GYRO_ROTATING_THRESHOLD = 25.0f;    // Gyroscope threshold for axis-specific rotation
    const float TILT_THRESHOLD = 0.2f;              // Threshold for tilt detection (20% of gravity)
    const float HORIZONTAL_THRESHOLD = 0.1f;        // Threshold for horizontal detection

    // 1. First, check if device is stationary (close to 1g acceleration and minimal rotation)
    if (fabs(accel_magnitude - g_acc) < ACCEL_STATIONARY_THRESHOLD && gyro_magnitude < 10.0f) {
        // Device is stationary, now check if it's horizontal or tilted

        // For horizontal stationary: acceleration should be mostly in Z direction
        // Z should be close to 9.8 m/s², X and Y should be close to 0
        if (fabs(data.ax) < HORIZONTAL_THRESHOLD * g_acc &&
            fabs(data.ay) < HORIZONTAL_THRESHOLD * g_acc &&
            fabs(data.az - g_acc) < HORIZONTAL_THRESHOLD * g_acc) {
            return Stationary;  // Horizontal stationary - 不亮灯
        }

        // For tilted stationary: significant horizontal acceleration component
        float horizontal_accel = sqrt(data.ax * data.ax + data.ay * data.ay);
        if (horizontal_accel > TILT_THRESHOLD * g_acc) {
            return Tilted_Stationary;  // Tilted stationary - 红灯
        }

        // Default to stationary if acceleration is close to gravity
        return Stationary;
    }

    // 2. Check for rotation (high gyroscope values)
    // Detect rotation around specific axes
    if (fabs(data.gx) > GYRO_ROTATING_THRESHOLD) {
        // If X-axis rotation is dominant
        if (fabs(data.gx) > fabs(data.gy) * 1.2f && fabs(data.gx) > fabs(data.gz) * 1.2f) {
            return X_Axis_Rotating;  // Rotation around X axis - 蓝灯
        }
    }

    if (fabs(data.gy) > GYRO_ROTATING_THRESHOLD) {
        // If Y-axis rotation is dominant
        if (fabs(data.gy) > fabs(data.gx) * 1.2f && fabs(data.gy) > fabs(data.gz) * 1.2f) {
            return Y_Axis_Rotating;  // Rotation around Y axis - 洋红灯
        }
    }

    if (fabs(data.gz) > GYRO_ROTATING_THRESHOLD) {
        // If Z-axis rotation is dominant
        if (fabs(data.gz) > fabs(data.gx) * 1.2f && fabs(data.gz) > fabs(data.gy) * 1.2f) {
            return Z_Axis_Rotating;  // Rotation around Z axis - 白灯
        }
    }

    // 3. Check for translation/movement in specific directions
    // We analyze linear acceleration by removing gravity component
    float lin_acc_x = data.ax;
    float lin_acc_y = data.ay;
    float lin_acc_z = data.az - g_acc;  // Remove gravity from Z axis

    // Calculate linear acceleration magnitude without gravity
    float lin_acc_mag = sqrt(lin_acc_x*lin_acc_x + lin_acc_y*lin_acc_y + lin_acc_z*lin_acc_z);

    // Check if there's significant linear acceleration
    if (lin_acc_mag > ACCEL_MOVING_THRESHOLD) {
        // Determine which axis has the most significant movement
        if (fabs(lin_acc_x) > fabs(lin_acc_y) * 1.2f && fabs(lin_acc_x) > fabs(lin_acc_z) * 1.2f) {
            return X_Axis_Moving;  // Movement along X axis - 黄灯
        }
        else if (fabs(lin_acc_y) > fabs(lin_acc_x) * 1.2f && fabs(lin_acc_y) > fabs(lin_acc_z) * 1.2f) {
            return Y_Axis_Moving;  // Movement along Y axis - 绿灯
        }
        else if (fabs(lin_acc_z) > fabs(lin_acc_x) * 1.2f && fabs(lin_acc_z) > fabs(lin_acc_y) * 1.2f) {
            return Z_Axis_Moving;  // Movement along Z axis - 青灯
        }
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

            if (msg_send(&msg, _led_pid) == 1) {
                const char* state_names[] = {
                    "Stationary", "X_Axis_Moving", "Y_Axis_Moving", "Z_Axis_Moving",
                    "X_Axis_Rotating", "Y_Axis_Rotating", "Z_Axis_Rotating", "Tilted_Stationary"
                };
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
            const char* state_names[] = {
                "Stationary", "X_Axis_Moving", "Y_Axis_Moving", "Z_Axis_Moving",
                "X_Axis_Rotating", "Y_Axis_Rotating", "Z_Axis_Rotating", "Tilted_Stationary"
            };
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
