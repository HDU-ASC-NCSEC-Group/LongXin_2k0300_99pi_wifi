#include "zf_common_headfile.h"
#include <math.h>

struct pwm_info servo_pwm_info;

float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

// 红色物块质心坐标
int16_t coordinate_x = 0;
int16_t coordinate_y = 0;

extern float real_x, real_y;  // 从坐标转换函数获取的物理坐标

// 机械臂相应数据
#define Arm_1     8.2f    // 机械臂第一个关节长度（单位：cm），根据实际情况调整
#define Arm_2     8.0f    // 机械臂第二个关节长度（单位：cm），根据实际情况调整
#define alpha     35.0f   // 角度1的起始位置与z轴的夹角
#define h         1.3f    // 抓取物块的高度（单位：cm），根据实际情况调整
#define delta_x   8.8f    // 摄像头与机械臂基座的水平距离（单位：cm），根据实际情况调整 8.8~9.0cm

// 机械臂四个转角
float angle_0 = 0.0f;   // 基座旋转角度
float angle_1 = 0.0f;   // 控制第一个机械臂
float angle_2 = 0.0f;   // 控制第二个机械臂
float angle_3 = 0.0f;   // 控制末端执行器（夹爪）开合程度

float len = sqrt(real_x * real_x + real_y * real_y);  // 目标点与机械臂基座的距离

void servo_test(void)
{
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty));

    float angle = atan2((int)(coordinate_y - UVC_HEIGHT / 2), (int)(coordinate_x - UVC_WIDTH / 2)) * 180.0 / CV_PI;  // 计算角度
    servo_motor_duty = angle;
    // 舵机活动范围限制
    if(servo_motor_duty >= SERVO_MOTOR_R_MAX)
    {
        servo_motor_duty = SERVO_MOTOR_R_MAX;
    }
    else if(servo_motor_duty <= SERVO_MOTOR_L_MAX)
    {
        servo_motor_duty = SERVO_MOTOR_L_MAX;
    }

}

// 根据摄像头获取的实际坐标解算angle_0、angle_1、angle_2、angle_3，控制机械臂夹取物块
void servo_control(void)
{
    angle_0 = atan2(real_x, real_y) * 180.0f / CV_PI;  // 基座旋转角度

    float L = len - delta_x;  // 目标点与机械臂基座的水平距离
    float H = h;              // 机械臂的高度

    // 计算得到angle_1和angle_2
    int Angle_1 = asin((pow(L,2.0) + pow(H,2.0) + pow(Arm_1,2.0) - pow(Arm_2,2.0)) / (2 * Arm_1 * sqrt(pow(L,2.0) + pow(H,2.0)))) * 180.0f / CV_PI - atan2(H, L) * 180.0f / CV_PI;  // 第一个机械臂的角度
    int Angle_2 = acos((pow(L,2.0) + pow(H,2.0) + pow(Arm_2,2.0) - pow(Arm_1,2.0)) / (2 * Arm_2 * sqrt(pow(L,2.0) + pow(H,2.0)))) * 180.0f / CV_PI - atan2(H, L) * 180.0f / CV_PI;  // 第二个机械臂的角度

    angle_1 = Angle_1 + alpha;  // 调整第一个机械臂的角度
    angle_2 = angle_1 + 90.0f - alpha - Angle_2;  // 调整第二个机械臂的角度

    /*
    设置一个标志位，刚开始，angle_3 = 0.0f 夹子打开到最大
    当机械臂移动到目标位置时，angle_3 = 180.0f 夹子关闭，抓取物块
    再写一个机械臂复位代码，形成一套动作链
    */
}
