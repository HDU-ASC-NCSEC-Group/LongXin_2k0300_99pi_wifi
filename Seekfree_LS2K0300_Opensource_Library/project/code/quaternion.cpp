#include "zf_common_headfile.h"
#include "quaternion.h"
#include <math.h>

// 滤波器参数
#define SAMPLE_TIME_MS    1.0f
#define DEG_TO_RAD        0.017453292519943f

static Quaternion attitude = {1.0f, 0.0f, 0.0f, 0.0f};
static uint8_t initialized = 0;

/**
 * @brief 初始化四元数模块
 */
void quaternion_init(void) {
    attitude.q0 = 1.0f;
    attitude.q1 = 0.0f;
    attitude.q2 = 0.0f;
    attitude.q3 = 0.0f;
    initialized = 1;
}

//imu963ra_gyro的转化函数
float imu963ra_gyro_transition(float gyro_value)
{
    float gyro_data = 0;
    switch(IMU963_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 131.0f;  break;              // 0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以 131           可以转化为带物理单位的数据，单位为：°/s
        case 0x08: gyro_data = (float)gyro_value / 65.5f;   break;              // 0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以 65.5          可以转化为带物理单位的数据，单位为：°/s
        case 0x10: gyro_data = (float)gyro_value / 32.8f;   break;              // 0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以 32.8          可以转化为带物理单位的数据，单位为：°/s
        case 0x18: gyro_data = (float)gyro_value / 16.4f;   break;              // 0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以 16.4          可以转化为带物理单位的数据，单位为：°/s
        default: break;
    }
    return gyro_data;
}

/**
 * @brief 更新四元数姿态
 */
void quaternion_update(void)
{
    if(!initialized) return;

    //获取原始陀螺仪数据并转换为弧度/秒（°/s）
    float gx = imu963ra_gyro_transition(imu963ra_gyro_x / 100 * 100) * DEG_TO_RAD;
    float gy = imu963ra_gyro_transition(imu963ra_gyro_y / 100 * 100) * DEG_TO_RAD;
    float gz = imu963ra_gyro_transition(imu963ra_gyro_z / 100 * 100) * DEG_TO_RAD;

    //一阶龙格-库塔法积分
    float dt = SAMPLE_TIME_MS / 1000.0f;
    float q0 = attitude.q0;
    float q1 = attitude.q1;
    float q2 = attitude.q2;
    float q3 = attitude.q3;

    // 四元数微分方程
    attitude.q0 += (-q1*gx - q2*gy - q3*gz) * (0.5f * dt);
    attitude.q1 += ( q0*gx - q3*gy + q2*gz) * (0.5f * dt);
    attitude.q2 += ( q3*gx + q0*gy - q1*gz) * (0.5f * dt);
    attitude.q3 += (-q2*gx + q1*gy + q0*gz) * (0.5f * dt);

    //四元数归一化
    float norm = sqrtf(attitude.q0 * attitude.q0 + attitude.q1 * attitude.q1 +
                       attitude.q2 * attitude.q2 + attitude.q3 * attitude.q3);

    norm = 1.0f / norm;
    attitude.q0 *= norm;
    attitude.q1 *= norm;
    attitude.q2 *= norm;
    attitude.q3 *= norm;

    //转换为欧拉角（弧度）
    attitude.roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * (1 / DEG_TO_RAD);
    attitude.pitch = asinf(2*(q0*q2 - q3*q1)) * (1 / DEG_TO_RAD);
    attitude.yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * (1 / DEG_TO_RAD);
}

Quaternion* get_eular_angles(void) 
{
    return &attitude;
}
