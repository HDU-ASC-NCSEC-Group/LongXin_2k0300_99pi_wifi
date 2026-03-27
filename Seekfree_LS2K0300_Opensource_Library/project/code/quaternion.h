#ifndef _quaternion_h_
#define _quaternion_h_

#include "zf_common_typedef.h"

// 四元数滤波器结构体
typedef struct {
    float q0, q1, q2, q3;       // 四元数
    float integralFBx, integralFBy, integralFBz; // 积分误差项
    float twoKp;                // 2 * 比例增益 (Kp)
    float twoKi;                // 2 * 积分增益 (Ki)
} quaternion_filter_t;

// 初始化滤波器，设置增益
void quaternion_init(float kp, float ki);

// 更新四元数（使用最新的加速度、陀螺仪、磁力计数据）
void quaternion_update(void);

// 获取欧拉角（弧度），结果通过指针返回
void quaternion_get_euler(float *roll, float *pitch, float *yaw);

#endif