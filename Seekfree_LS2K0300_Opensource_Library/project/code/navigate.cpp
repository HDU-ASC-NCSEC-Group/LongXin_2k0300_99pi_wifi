#include "zf_common_headfile.h"

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

    int16_t pwm_1 = 0, pwm_2 = 0, pwm_3 = 0, pwm_4 = 0;

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
        pwm_1 = 20; pwm_2 = 20;    // 直行（这里改成你想要的速度）
        pwm_3 = pwm_2; pwm_4 = pwm_1;
        flag_avoid = 0;
    }
    else
    {
        float avg_angle = angle_sum / calculation_angle_cnt;

        // avg_angle < 0 → 障碍物偏左 → 需向右转
        if(avg_angle < 0)
        {
            pwm_1 =  20; pwm_2 =  20;   // 左侧轮前进
            pwm_3 = -pwm_2; pwm_4 = -pwm_1;   // 右侧轮后退（差速右转）
            flag_avoid = 1;             // 右转
        }
        else                             // avg_angle > 0 → 偏右 → 左转
        {
            pwm_1 = -20; pwm_2 = -20;   // 左侧轮后退
            pwm_3 = -pwm_2; pwm_4 = -pwm_1;   // 右侧轮前进（差速左转）
            flag_avoid = 2;             // 左转
        }
    }

    Motor_Set(1, pwm_1);
    Motor_Set(4, pwm_4);
    Motor_Set(2, pwm_2);
    Motor_Set(3, pwm_3);

    text_cnt = calculation_angle_cnt;
    text_angle = angle_sum;
    printf("避障状态：%d\n", flag_avoid);
}
