#ifndef __navigate_h
#define __navigate_h

#include "zf_common_headfile.h"

// 角度位置环PID结构体（串级PID的外环）
typedef struct {
    float target;           // 目标角度
    float actual;           // 实际角度
    float output;           // 输出（目标转向速度）
    
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    
    float error;            // 当前误差
    float last_error;       // 上次误差
    float error_integral;   // 误差积分
    float integral_max;     // 积分限幅
    
    float OutMax;           // 最大输出（最大转向速度）
    float OutMin;           // 最小输出（最小转向速度）
} Angle_Position_PID;

extern int16_t LeftPWM, RightPWM;         // 左右电机的PWM值
extern uint8_t is_angle_turning;  // 是否正在执行转向动作的标志


// 雷达避障函数
void avoid(void);

// 转向控制函数
void Start_Angle_Turn(float angle);
void Stop_Angle_Turn(void);
uint8_t Update_Angle_Turn(void);
uint8_t Is_Angle_Turning(void);
float Get_Angle_Turn_Error(void);

// uwb导航函数


#endif