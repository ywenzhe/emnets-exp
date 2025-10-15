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
struct MPU6050Data
{
    float ax, ay, az;
    float gx, gy, gz;
};
enum MoveState{Stationary, Tilted, Rotating, Moving};

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
    (void) arg;
    LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
    led.change_led_color(0);
    while(1){
        // Input your codes
        // Wait for a message to control the LED
        // Display different light colors based on the motion state of the device.
    }
    return NULL;
}

#define g_acc (9.8)
MoveState detectMovement(MPU6050Data &data)
{
    // Input your code
    // Please use your imagination or search online 
    // to determine the motion state of the device 
    // based on the data obtained from the MPU6050 sensor.
}

void *_imu_thread(void *arg)
{
    (void) arg;
    // Input your code
    // 1. initial mpu6050 sensor
    // 2. Acquire sensor data every 100ms
    // 3. Determine the motion state
    // 4. notify the LED thread to display the light color through a message.
    return NULL;
}
static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

int main(void)
{
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
    while(1)
    {

    }
    return 0;
}
