# 双线程LED控制和IMU传感器实验

## 项目概述

本项目实现了基于RIOT操作系统的双线程应用，一个线程用于LED控制，另一个线程用于读取和处理MPU6050传感器数据。系统能够根据设备的运动状态（水平静止、倾斜静止、旋转、平移）显示不同颜色的LED灯。

## 硬件连接

### LED连接
- RGB LED的三个引脚分别连接到：
  - 红色(R): GPIO26
  - 绿色(G): GPIO25
  - 蓝色(B): GPIO27
- LED GND连接到GND
- LED VCC连接到3V3或5V

### MPU6050传感器连接
- 使用I2C接口连接
- SDA引脚：GPIO21
- SCL引脚：GPIO22
- VCC连接到3.3V
- GND连接到GND

## 编译和烧写

```bash
sudo chmod 777 /dev/ttyUSB0
make BOARD=esp32-wroom-32 flash term
```

## 代码实现详解

### 1. 数据结构定义

```cpp
struct MPU6050Data {
    float ax, ay, az;  // 三轴加速度数据 (m/s²)
    float gx, gy, gz;  // 三轴陀螺仪数据 (°/s)
};

enum MoveState {
    Stationary,  // 水平静止
    Tilted,      // 倾斜静止
    Rotating,    // 旋转
    Moving       // 平移运动
};
```

### 2. LED控制线程 (_led_thread)

**位置**: `main.cpp:44-75`

**功能实现**:
- 初始化RGB LED控制器，初始状态为熄灭
- 使用消息队列接收来自IMU线程的状态变化通知
- 根据不同的运动状态设置对应的LED颜色

**颜色映射**:
- 水平静息 (Stationary): LED熄灭 (COLOR_NONE)
- 倾斜静止 (Tilted): 红色 (COLOR_RED)
- 旋转 (Rotating): 蓝色 (COLOR_BLUE)
- 平移运动 (Moving): 绿色 (COLOR_GREEN)

**关键代码**:
```cpp
void* _led_thread(void* arg) {
    LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
    led.change_led_color(COLOR_NONE);  // 初始关闭LED

    msg_t msg;
    while (1) {
        msg_receive(&msg);  // 等待消息

        switch(msg.type.value) {
            case Stationary: led.change_led_color(COLOR_NONE); break;
            case Tilted: led.change_led_color(COLOR_RED); break;
            case Rotating: led.change_led_color(COLOR_BLUE); break;
            case Moving: led.change_led_color(COLOR_GREEN); break;
        }
    }
}
```

### 3. 运动状态检测函数 (detectMovement)

**位置**: `main.cpp:78-124`

**功能实现**: 根据MPU6050传感器数据智能判断设备的运动状态

**检测算法**:
1. **静止检测**: 检查加速度大小是否接近重力加速度(9.8m/s²)，且陀螺仪数值较小
2. **水平检测**: 分析加速度分量，如果主要在Z轴方向且X、Y轴接近零，则为水平静止
3. **倾斜检测**: 计算水平加速度分量，如果超过阈值则为倾斜静止
4. **旋转检测**: 陀螺仪数值超过阈值时判断为旋转
5. **平移检测**: 加速度大小明显偏离重力加速度时判断为平移运动

**关键阈值参数**:
- `ACCEL_STATIONARY_THRESHOLD = 1.2f`: 静止检测的加速度阈值
- `ACCEL_MOVING_THRESHOLD = 2.5f`: 运动检测的加速度阈值
- `GYRO_ROTATING_THRESHOLD = 30.0f`: 旋转检测的陀螺仪阈值
- `TILT_THRESHOLD = 0.2f`: 倾斜检测阈值(20%重力加速度)
- `HORIZONTAL_THRESHOLD = 0.1f`: 水平检测阈值(10%重力加速度)

**关键代码**:
```cpp
MoveState detectMovement(MPU6050Data& data) {
    float accel_magnitude = sqrt(data.ax*data.ax + data.ay*data.ay + data.az*data.az);
    float gyro_magnitude = sqrt(data.gx*data.gx + data.gy*data.gy + data.gz*data.gz);

    // 静止检测
    if (fabs(accel_magnitude - g_acc) < ACCEL_STATIONARY_THRESHOLD &&
        gyro_magnitude < 15.0f) {

        // 水平检测
        if (fabs(data.ax) < HORIZONTAL_THRESHOLD * g_acc &&
            fabs(data.ay) < HORIZONTAL_THRESHOLD * g_acc &&
            fabs(data.az - g_acc) < HORIZONTAL_THRESHOLD * g_acc) {
            return Stationary;
        }

        // 倾斜检测
        float horizontal_accel = sqrt(data.ax*data.ax + data.ay*data.ay);
        if (horizontal_accel > TILT_THRESHOLD * g_acc) {
            return Tilted;
        }

        return Stationary;
    }

    // 旋转检测
    if (gyro_magnitude > GYRO_ROTATING_THRESHOLD) {
        return Rotating;
    }

    // 平移检测
    if (fabs(accel_magnitude - g_acc) > ACCEL_MOVING_THRESHOLD) {
        return Moving;
    }

    return Stationary;
}
```

### 4. IMU传感器线程 (_imu_thread)

**位置**: `main.cpp:126-200`

**功能实现**:
- 初始化MPU6050传感器
- 配置传感器参数(量程、采样率、滤波器)
- 每100ms读取一次传感器数据
- 转换原始数据为物理单位
- 调用运动检测函数判断设备状态
- 通过消息机制通知LED线程状态变化

**传感器配置**:
- 加速度计量程: ±2g (MPU6050_ACCEL_FS_2)
- 陀螺仪量程: ±250°/s (MPU6050_GYRO_FS_250)
- 采样率: 100Hz
- 数字低通滤波器: 20Hz带宽 (MPU6050_DLPF_BW_20)

**数据转换**:
- 加速度: 原始值 / 16384.0 × 9.8 = m/s²
- 陀螺仪: 原始值 / 131.0 = °/s

**关键代码**:
```cpp
void* _imu_thread(void* arg) {
    MPU6050 mpu(MPU6050_IIC_DEV, MPU6050_DEFAULT_ADDRESS);

    // 初始化和配置
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setRate(100);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);

    MPU6050Data sensorData;
    MoveState currentState, previousState = Stationary;
    msg_t msg;

    while (1) {
        // 读取传感器数据
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // 数据转换
        sensorData.ax = ax_raw / 16384.0f * g_acc;
        sensorData.ay = ay_raw / 16384.0f * g_acc;
        sensorData.az = az_raw / 16384.0f * g_acc;
        sensorData.gx = gx_raw / 131.0f;
        sensorData.gy = gy_raw / 131.0f;
        sensorData.gz = gz_raw / 131.0f;

        // 状态检测
        currentState = detectMovement(sensorData);

        // 状态变化通知
        if (currentState != previousState) {
            msg.type.value = (uint32_t)currentState;
            msg_send(&msg, _led_pid);
            previousState = currentState;
        }

        delay_ms(100);  // 100ms采样间隔
    }
}
```

### 5. LED控制器类完善 (LEDController)

**文件**: `ledcontroller.cpp`

**构造函数实现**:
- 存储RGB三个GPIO引脚
- 将引脚初始化为输出模式
- 初始状态关闭所有LED
- 初始化内部RGB状态数组

**change_led_color函数实现**:
- 支持8种颜色显示(红、绿、蓝、黄、青、洋红、白、关闭)
- 使用switch语句根据枚举值设置RGB分量
- 通过gpio_write函数控制实际GPIO引脚状态

## 线程间通信机制

系统使用RIOT的消息传递机制实现线程间通信：

1. **消息类型**: 使用`msg_t`结构体传递运动状态信息
2. **传递方向**: IMU线程 → LED线程
3. **触发条件**: 仅在运动状态发生变化时发送消息
4. **数据内容**: 消息的`type.value`字段包含当前的运动状态枚举值

## 系统特性

- **实时性**: 100ms采样间隔，快速响应运动状态变化
- **精确性**: 使用物理单位和合理阈值进行状态判断
- **低功耗**: 仅在状态变化时更新LED，减少不必要的操作
- **调试友好**: 定期输出传感器数据和状态信息
- **容错性**: 包含传感器连接失败检测和错误处理

## 预期行为

- 设备水平放置时：LED熄灭
- 设备倾斜但静止时：LED显示红色
- 设备旋转时：LED显示蓝色
- 设备平移运动时：LED显示绿色
- 状态切换时：串口输出状态变化信息

这个项目展示了如何在嵌入式系统中实现多线程协作、传感器数据处理和硬件控制的完整解决方案。