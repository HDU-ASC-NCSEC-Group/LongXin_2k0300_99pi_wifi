#include "zf_common_headfile.h"
#include <math.h>

// -----------------------------------------------------------------------------------
// 变量申明
uint8_t text_cnt = 0;
float text_angle = 0;

//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// 函数简介     数据处理函数，小车前进避障，只需要前方点的数据
// 使用示例     data_process();
//------------------------------------------------------------------------------------

void avoid(void)
{
    volatile uint8_t i = 0;
    volatile uint8_t calculation_angle_cnt = 0;
    float angle_sum = 0;
    uint8_t flag_avoid = 0;
    static uint16_t distance = 330;          // 避障阈值 mm

    const float FRONT_ANGLE = 270.0f;        // ★ 雷达 0° 对应的方向？这里按正前方 = 270° 设定

    for(i = 0; i < 50; i++)
    {
        uint16_t d = PointDataProcess[i].distance;
        if(d > 0 && d < distance)
        {
            float raw_angle = PointDataProcess[i].angle;

            // 1. 计算相对于正前方的偏离角（范围 -180° ~ 180°）
            float diff = raw_angle - FRONT_ANGLE;
            if(diff > 180.0f)       diff -= 360.0f;
            else if(diff < -180.0f)  diff += 360.0f;

            // 2. 累加偏离角（左侧为负，右侧为正）
            angle_sum += diff;
            calculation_angle_cnt++;
        }
    }

    if(calculation_angle_cnt == 0)          // 无障碍
    {
        Motor_Move_Straight(2000);
        flag_avoid = 0;
    }
    else
    {
        float avg_angle = angle_sum / calculation_angle_cnt;

        // avg_angle < 0 → 障碍物偏左 → 需向右转
        if(avg_angle < 0)
        {
            Motor_Spot_Right(3600);
            flag_avoid = 1;             // 右转
        }
        else                             // avg_angle > 0 → 偏右 → 左转
        {
            Motor_Spot_Left(3600);
            flag_avoid = 2;             // 左转
        }
    }

    text_cnt = calculation_angle_cnt;
    text_angle = angle_sum;
    printf("避障状态：%d\n", flag_avoid);
}

//====================================================================================

// -----------------------------------------------------------------------------------
// 函数简介     转向函数，执行转向动作
// 参数说明     
// 使用示例     
// -----------------------------------------------------------------------------------
int16_t turn_difpwm = 0;               // 转向差速PWM值
int16_t LeftPWM, RightPWM = 0;         // 左右电机的PWM值

uint8_t is_angle_turning = 0;          // 是否正在执行转向动作的标志
float target_angle_increment = 0.0f;   // 目标角度增量
float initial_yaw = 0.0f;              // 转向开始时的初始角度

// ------------------ 角度位置PID结构体（单级） ------------------
Angle_Position_PID angle_pos_pid = {
    .Kp = 1250.0f,       // 比例系数，需根据实际调试
    .Ki = 0.0f,       // 积分系数
    .Kd = 0.0f,       // 微分系数

    .OutMax = 7000.0f,    // 最大差速PWM输出
    .OutMin = -7000.0f,   // 最小差速PWM输出
};

// ------------------ 内部PID计算 ------------------
static void AnglePositionPID_Update(Angle_Position_PID *pid)
{
    // 计算角度误差，归一化到 [-180, 180]
    pid->error = pid->target - pid->actual;
    while (pid->error > 180.0f)  pid->error -= 360.0f;
    while (pid->error < -180.0f) pid->error += 360.0f;

    // 积分项（带限幅）
    if (pid->Ki != 0) {
        pid->error_integral += pid->error;
        if (pid->error_integral > pid->integral_max)
            pid->error_integral = pid->integral_max;
        if (pid->error_integral < -pid->integral_max)
            pid->error_integral = -pid->integral_max;
    }

    // 微分项（也需归一化，防止yaw环绕时error跳变导致D项尖峰）
    float error_diff = pid->error - pid->last_error;
    while (error_diff > 180.0f)  error_diff -= 360.0f;
    while (error_diff < -180.0f) error_diff += 360.0f;
    pid->last_error = pid->error;

    // PID输出
    pid->output = pid->Kp * pid->error
                + pid->Ki * pid->error_integral
                + pid->Kd * error_diff;

    // 输出限幅
    if (pid->output > pid->OutMax) pid->output = pid->OutMax;
    if (pid->output < pid->OutMin) pid->output = pid->OutMin;
}

// ------------------ 应用差速到电机 ------------------
static void Apply_Differential_Steer(float diff_pwm)
{
    // 直接设置差速，同时保留平均速度（直行分量由其他任务控制）
    turn_difpwm = (int16_t)diff_pwm;
    LeftPWM = -turn_difpwm / 2;
    RightPWM = turn_difpwm / 2;
}

// ------------------ 开始角度转向任务 ------------------
// >0 右转（顺时针） <0 左转（逆时针）
void Start_Angle_Turn(float angle)
{
    if (is_angle_turning) {
        Stop_Angle_Turn();
    }

    is_angle_turning = 1;
    target_angle_increment = angle;
    initial_yaw = Yaw_Result;

    // 初始化位置PID目标
    angle_pos_pid.target = initial_yaw + angle;
    while (angle_pos_pid.target > 180.0f)  angle_pos_pid.target -= 360.0f;
    while (angle_pos_pid.target < -180.0f) angle_pos_pid.target += 360.0f;
    angle_pos_pid.actual = initial_yaw;
    angle_pos_pid.error = 0;
    angle_pos_pid.last_error = 0;
    angle_pos_pid.error_integral = 0;
    angle_pos_pid.integral_max = 100.0f;   // 积分上限，可调

    // 初始化差速为0，避免突变
    Apply_Differential_Steer(0.0f);
}

// ------------------ 停止角度转向任务 ------------------
void Stop_Angle_Turn(void)
{
    is_angle_turning = 0;
    angle_pos_pid.error_integral = 0;       // 清空积分
    angle_pos_pid.output = 0.0f;           // 清空输出
    Apply_Differential_Steer(0.0f);         // 差速回零，停止转向
}

// ------------------ 转向任务更新（中断中周期调用） ------------------
uint8_t Update_Angle_Turn(void)
{
    if (!is_angle_turning) return 1;

    // 更新当前角度
    angle_pos_pid.actual = Yaw_Result;

    // 计算位置PID
    AnglePositionPID_Update(&angle_pos_pid);

    // 位置环输出直接作为差速PWM，负号用于匹配方向（可现场调整）
    float diff_output = -angle_pos_pid.output;   // 负号取决于你的电机接线
    Apply_Differential_Steer(diff_output);

    // 检查是否到达目标角度（带死区）
    float angle_error = fabsf(angle_pos_pid.target - Yaw_Result);
    if (angle_error < 1.0f) {   // 1度误差内认为完成
        Stop_Angle_Turn();
        return 1;               // 转向完成
    }

    return 0;                   // 转向中
}

// ------------------ 状态查询 ------------------
uint8_t Is_Angle_Turning(void)
{
    return is_angle_turning;
}

float Get_Angle_Turn_Error(void)
{
    return angle_pos_pid.error;
}