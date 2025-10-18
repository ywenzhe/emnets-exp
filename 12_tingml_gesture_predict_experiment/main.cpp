#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "shell.h"
#include <log.h>
#include <xtimer.h>
#include "ledcontroller.hh"
#include "ztimer.h"
#include "mpu6050.h"
#include <string>
#include "msg.h"

void setup();
int predict(float *imu_data, int data_len, float threashold, int class_num);
using namespace std;
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack_for_motion_thread[THREAD_STACKSIZE];
static char stack_for_led_thread[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
#define LED_GPIO_R GPIO26
#define LED_GPIO_G GPIO25
#define LED_GPIO_B GPIO27

#define g_acc (9.8)
#define SAMPLES_PER_GESTURE (20)

struct MPU6050Data
{
    float ax, ay, az; // acceler_x_axis, acceler_y_axis, acceler_z_axis
    float gx, gy, gz; // gyroscope_x_axis, gyroscope_y_axis, gyroscope_z_axis
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

void delay_ms(uint32_t sleep_ms)
{
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}
/**
 * LED control thread function.
 * Then, it enters an infinite loop where it waits for messages to control the LED.
 * @param arg Unused argument.
 * @return NULL.
 */
void *_led_thread(void *arg)
{
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

float gyro_fs_convert = 1.0;
float accel_fs_convert;

void get_imu_data(MPU6050 mpu, float *imu_data){
    int16_t ax, ay, az, gx, gy, gz;
    for(int i = 0; i < SAMPLES_PER_GESTURE; ++i)
    {
        /* code */
        delay_ms(20);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        imu_data[i*6 + 0] = ax / accel_fs_convert;
        imu_data[i*6 + 1] = ay / accel_fs_convert;
        imu_data[i*6 + 2] = az / accel_fs_convert;
        imu_data[i*6 + 3] = gx / gyro_fs_convert;
        imu_data[i*6 + 4] = gy / gyro_fs_convert;
        imu_data[i*6 + 5] = gz / gyro_fs_convert;
    }
} 

void *_motion_thread(void *arg)
{
    (void) arg;
    // Initialize MPU6050 sensor
    MPU6050 mpu;
    // get mpu6050 device id
    uint8_t device_id = mpu.getDeviceID();
    printf("[IMU_THREAD] DEVICE_ID:0x%x\n", device_id);
    mpu.initialize();

    // Configure gyroscope and accelerometer full scale ranges
    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
    uint16_t accel_fs_real = 1;

    // Convert gyroscope full scale range to conversion factor
    if (gyro_fs == MPU6050_GYRO_FS_250)
        gyro_fs_convert = 131.0;
    else if (gyro_fs == MPU6050_GYRO_FS_500)
        gyro_fs_convert = 65.5;
    else if (gyro_fs == MPU6050_GYRO_FS_1000)
        gyro_fs_convert = 32.8;
    else if (gyro_fs == MPU6050_GYRO_FS_2000)
        gyro_fs_convert = 16.4;
    else
        printf("[IMU_THREAD] Unknown GYRO_FS: 0x%x\n", gyro_fs);

    // Convert accelerometer full scale range to real value
    if (accel_fs_g == MPU6050_ACCEL_FS_2)
        accel_fs_real = g_acc * 2;
    else if (accel_fs_g == MPU6050_ACCEL_FS_4)
        accel_fs_real = g_acc * 4;
    else if (accel_fs_g == MPU6050_ACCEL_FS_8)
        accel_fs_real = g_acc * 8;
    else if (accel_fs_g == MPU6050_ACCEL_FS_16)
        accel_fs_real = g_acc * 16;
    else
        printf("[IMU_THREAD] Unknown ACCEL_FS: 0x%x\n", accel_fs_g);

    // Calculate accelerometer conversion factor
    accel_fs_convert = 32768.0 / accel_fs_real;
    float imu_data[SAMPLES_PER_GESTURE * 6] = {0};
    int data_len = SAMPLES_PER_GESTURE * 6;
    delay_ms(100);
    // Main loop
    int predict_interval_ms = 100;
    int ret = 0;
#define class_num (8)
    float threshold = 0.7;
    MoveState motions[class_num] = {Stationary, X_Axis_Moving, Y_Axis_Moving, Z_Axis_Moving, X_Axis_Rotating, Y_Axis_Rotating, Z_Axis_Rotating, Tilted_Stationary};
    
    MoveState currentState, previousState = Stationary;
    msg_t msg;

    while (1) {
        delay_ms(predict_interval_ms); 
        // Read sensor data
        get_imu_data(mpu, imu_data);
        ret = predict(imu_data, data_len, threshold, class_num);
        // tell the led thread to do some operations
        // input your code

        currentState = motions[ret];

        if(currentState != previousState) {
            msg.type = (uint32_t)currentState;
            msg_send(&msg, _led_pid);
        }

        previousState = currentState;

        // Print result
        //printf("Predict: %d, %s\n", ret, motions[ret].c_str());
    }
    return NULL;
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    setup();
    _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _led_thread, NULL,
                            "led_controller_thread");
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    thread_create(stack_for_motion_thread, sizeof(stack_for_motion_thread), THREAD_PRIORITY_MAIN - 1,
                            THREAD_CREATE_STACKTEST, _motion_thread, NULL,
                            "imu_read_thread");
    printf("[Main] Initialization successful - starting the shell now\n");
    while(1);
    return 0;
    
}
