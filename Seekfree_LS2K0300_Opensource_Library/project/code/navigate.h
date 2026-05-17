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

//============================================================
// UWB跟随参数设置
//============================================================
#define FOLLOW_BASE_SPEED            2500    // 基础前进速度
#define FOLLOW_TURN_GAIN             1000    // 转向修正系数 (每度方位角减速量)
#define FOLLOW_MAX_DIST_M            5.0f    // 最大有效距离（m），超过此距离认为无目标
#define FOLLOW_MIN_DIST_M            0.3f    // 最小有效距离（m），低于此距离认为过近
#define FOLLOW_AZIMUTH_DEADZONE      5.0f    // 方位角死区（度），在此范围内不进行转向修正

// 雷达避障参数
#define FOLLOW_OBS_NEAR_MM           200     // 紧急避障距离（mm），原地转向
#define FOLLOW_OBS_AVOID_MM          400     // 常规避障距离（mm），正常避障
#define FOLLOW_OBS_ANGLE_RANGE       50.0f   // 前方避障角度范围（+-度）

// 超时保护
#define FOLLOW_UWB_TIMEOUT_CNT       200       // UWB 超时计数值 (×10ms)，超时停机

// 雷达避障函数
void avoid(void);

// 转向控制函数
void Start_Angle_Turn(float angle);
void Stop_Angle_Turn(void);
uint8_t Update_Angle_Turn(void);
uint8_t Is_Angle_Turning(void);
float Get_Angle_Turn_Error(void);

// uwb导航函数
void uwb_follow(void);


#endif