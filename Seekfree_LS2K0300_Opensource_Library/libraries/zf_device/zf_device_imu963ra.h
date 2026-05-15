#ifndef _zf_device_imu963ra_h
#define _zf_device_imu963ra_h

#include "zf_common_typedef.h"

extern int16 imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z;  
extern int16 imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z;
extern int16 imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU963RA 加速度计数据转换为实际物理数据
// 参数说明     acc_value       任意轴的加速度计数据
// 返回参数     void
// 使用示例     float data = imu963ra_acc_transition(imu963ra_acc_x);           // 单位为 g(m/s^2)
// 备注信息     +- 8G
//-------------------------------------------------------------------------------------------------------------------
#define imu963ra_acc_transition(acc_value)      ((float)acc_value / 4098)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU963RA 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float data = imu963ra_gyro_transition(imu963ra_gyro_x);         // 单位为 °/s
// 备注信息     +- 2000dps
//-------------------------------------------------------------------------------------------------------------------
#define imu963ra_gyro_transition(gyro_value)    ((float)gyro_value / 14.3f)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU963RA 磁力计数据转换为实际物理数据
// 参数说明     mag_value       任意轴的磁力计数据
// 返回参数     void
// 使用示例     float data = imu963ra_mag_transition(imu963ra_mag_x);           // 单位为 G
// 备注信息     +- 8G
//-------------------------------------------------------------------------------------------------------------------
#define imu963ra_mag_transition(mag_value)    ((float)mag_value / 3000)

void imu963ra_get_acc(void);
void imu963ra_get_gyro(void);
void imu963ra_get_mag(void);

#endif