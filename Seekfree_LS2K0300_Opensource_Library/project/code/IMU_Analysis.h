/********************************************************************************************************************
* IMU姿态解算
********************************************************************************************************************/
#ifndef __IMU_ANALYSIS_H__
#define __IMU_ANALYSIS_H__

// 全局变量声明
extern volatile float Yaw_Result;    // 偏航角（Yaw）
extern volatile float Roll_Result;   // 横滚角（Roll）
extern volatile float Pitch_Result;  // 俯仰角（Pitch）

// IMU963RA 通信+解析 使能标志位
extern volatile uint8_t IMU963RA_analysis_enable;

// 函数声明
void    IMU963RA_Calibration_Start      (void);
uint8_t IMU963RA_Calibration_Check      (void);
void    IMU963RA_AHRS_Update            (void);

#endif