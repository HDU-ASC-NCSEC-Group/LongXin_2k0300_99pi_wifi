/********************************************************************************************************************
* IMU姿态解算
********************************************************************************************************************/
#include "zf_device_imu963ra.h"
#include "MahonyAHRS.h"
#include <math.h>


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 获取原始数据
// 使用示例     imu963ra_get_data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
// imu963ra_mag_x           imu963ra_mag_y          imu963ra_mag_z  
void imu963ra_get_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();
}

// 条件编译选项
#define ENABLE_FULL_EULER    1   // 1: 计算全部欧拉角, 0: 只计算Yaw角

// 全局变量
volatile float Yaw_Result = 0.0f;    // 偏航角（Yaw）
volatile float Roll_Result = 0.0f;   // 横滚角（Roll）
volatile float Pitch_Result = 0.0f;  // 俯仰角（Pitch）

// IMU963RA 分析使能标志位
volatile uint8_t IMU963RA_analysis_enable = 0;

// 传感器数据缩放因子
#define GYRO_SCALE    (2000.0f / 32768.0f * M_PI / 180.0f)  // 陀螺仪刻度因子（转换为rad/s）
#define ACC_SCALE     (8.0f / 32768.0f * 9.81f)            // 加速度计刻度因子（转换为m/s²）
#define MAG_SCALE     (4912.0f / 32768.0f)                 // 磁力计刻度因子（转换为uT）


/*******************************************************************************************************************/
/*[S] 零飘校准 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 校准需要的样本数
#define CALIB_TARGET_SAMPLES    400  

// 枚举定义动态校准状态
typedef enum {
    CALIB_STATE_SPARE   = 0,     				// c
    CALIB_STATE_RUNNING = 1,   					// 校准中
    CALIB_STATE_DONE    = 2       				// 已校准
} CalibState_t;

static CalibState_t calib_state = CALIB_STATE_SPARE;
static uint16_t calib_count = 0;
static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
static int32_t sum_mx = 0, sum_my = 0, sum_mz = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;
static float mag_off_x = 0, mag_off_y = 0, mag_off_z = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 校准初始化
// 使用示例     IMU963RA_Calibration_Start();                                              // 开始校准
// 备注信息     校准状态机启用前的操作，用于初始化校准状态
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Calibration_Start(void)
{
    calib_state = CALIB_STATE_RUNNING;
    calib_count = 0;
    sum_gx = 0;
    sum_gy = 0;
    sum_gz = 0;
    sum_mx = 0;
    sum_my = 0;
    sum_mz = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 校准状态机
// 使用示例     IMU963RA_Calibration_Check();                                              // 检查校准状态
// 返回校准状态（数字对应CalibState_t的枚举定义）
// 0 未校准
// 1 校准中
// 2 已校准
// 备注信息     调用方法特殊，可以参考调试文件的代码调用
//-------------------------------------------------------------------------------------------------------------------
int8_t IMU963RA_Calibration_Check(void)
{
    if(calib_state == CALIB_STATE_DONE)
    {
        return 2;
    }

    if(calib_state == CALIB_STATE_SPARE)
    {
        return 0;
    }
    
    // 检查是否允许收集数据
    if(IMU963RA_analysis_enable)
    {
        // 收集数据
        sum_gx += imu963ra_gyro_x;
        sum_gy += imu963ra_gyro_y;
        sum_gz += imu963ra_gyro_z;
        sum_mx += imu963ra_mag_x;
        sum_my += imu963ra_mag_y;
        sum_mz += imu963ra_mag_z;
        calib_count++;

        IMU963RA_analysis_enable = 0;
        
        if(calib_count >= CALIB_TARGET_SAMPLES)
        {
            gyro_off_x = (float)sum_gx / CALIB_TARGET_SAMPLES;
            gyro_off_y = (float)sum_gy / CALIB_TARGET_SAMPLES;
            gyro_off_z = (float)sum_gz / CALIB_TARGET_SAMPLES;
            mag_off_x  = (float)sum_mx / CALIB_TARGET_SAMPLES;
            mag_off_y  = (float)sum_my / CALIB_TARGET_SAMPLES;
            mag_off_z  = (float)sum_mz / CALIB_TARGET_SAMPLES;
            
            calib_state = CALIB_STATE_DONE;
        }
    }
    
    return 1;
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 零飘校准 [E]*/
/*******************************************************************************************************************/

/*******************************************************************************************************************/
/*[S] 姿态解算 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转欧拉角
// 使用示例     QuaternionToEuler();                                              // 四元数转欧拉角
// 备注信息     将MahonyAHRS的四元数结果转换为欧拉角
//-------------------------------------------------------------------------------------------------------------------
void QuaternionToEuler(void)
{
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    
    #if ENABLE_FULL_EULER
    // 计算Roll（横滚角）
    Roll_Result = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3) * 180.0f / M_PI;
    
    // 计算Pitch（俯仰角）
    Pitch_Result = asinf(-2.0f * (q1q3 - q0q2)) * 180.0f / M_PI;
    #endif
    
    // 计算Yaw（偏航角）
    Yaw_Result = atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3) * 180.0f / M_PI;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 姿态解算
// 使用示例     IMU963RA_AHRS_Update();                                              // 执行姿态解算
// 备注信息     定时调用该函数，更新姿态解算结果
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_AHRS_Update(void)
{
    // 只有在校准完成后才执行姿态解算
    if (calib_state != CALIB_STATE_DONE)
    {
        return;  // 未校准完成，不执行解算
    }

    // 获取原始数据
    imu963ra_get_data();
    
    // 应用零飘校准
    float gx = (imu963ra_gyro_x - gyro_off_x) * GYRO_SCALE;
    float gy = (imu963ra_gyro_y - gyro_off_y) * GYRO_SCALE;
    float gz = (imu963ra_gyro_z - gyro_off_z) * GYRO_SCALE;
    
    float ax = imu963ra_acc_x * ACC_SCALE;
    float ay = imu963ra_acc_y * ACC_SCALE;
    float az = imu963ra_acc_z * ACC_SCALE;
    
    float mx = (imu963ra_mag_x - mag_off_x) * MAG_SCALE;
    float my = (imu963ra_mag_y - mag_off_y) * MAG_SCALE;
    float mz = (imu963ra_mag_z - mag_off_z) * MAG_SCALE;
    
    // 调用Mahony AHRS算法
    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
    
    // 四元数转欧拉角
    QuaternionToEuler();
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 姿态解算 [E]*/
/*******************************************************************************************************************/
