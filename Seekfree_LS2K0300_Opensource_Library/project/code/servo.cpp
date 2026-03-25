#include "zf_common_headfile.h"
#include <math.h>

struct pwm_info servo_pwm_info;

float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

// 红色物块质心坐标
int16_t coordinate_x = 0;
int16_t coordinate_y = 0;

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

    //逐飞的舵机测试
    // if(servo_motor_dir)
    // {
    //     servo_motor_duty ++;
    //     if(servo_motor_duty >= SERVO_MOTOR_R_MAX)
    //     {
    //         servo_motor_dir = 0x00;
    //     }
    // }
    // else
    // {
    //     servo_motor_duty --;
    //     if(servo_motor_duty <= SERVO_MOTOR_L_MAX)
    //     {
    //         servo_motor_dir = 0x01;
    //     }
    // }
}

void servo_pid(void)
{

}
