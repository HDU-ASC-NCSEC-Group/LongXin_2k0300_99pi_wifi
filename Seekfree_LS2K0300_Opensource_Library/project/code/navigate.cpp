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
    volatile uint8_t calculation_angle_cnt = 0; // 用于判断50个点中需要做避障的点
    // volatile float angle;  // 合角，计算所有需要避障的点合成一个运动角度
    float angle_sum = 0;
    uint8_t flag_avoid = 0; // 避障标志位，0不避障，1右，2左
    static uint16_t distance = 280; // 避障距离阈值，单位 mm

    for(i = 0; i < 50; i ++)
    {
        if(0 < PointDataProcess[i].distance && PointDataProcess[i].distance < distance)
        {
            if(PointDataProcess[i].angle<50)
				angle_sum += PointDataProcess[i].angle;
			
			else 
				angle_sum += (PointDataProcess[i].angle-360);
			calculation_angle_cnt++;
        }
    }

    // 不避障
    if(calculation_angle_cnt == 0)
    {
        Motor_Move_Straight(20);

        flag_avoid = 0;
    }

    // 需要避障
    else
    {
        // 向右
        if((angle_sum/calculation_angle_cnt) > 0)
        {
            Motor_Spot_Right(15);

            flag_avoid = 1;
        }
        // 向左
        else
        {
            Motor_Spot_Left(15);

            flag_avoid = 2;
        }
    }

    text_cnt = calculation_angle_cnt;
	text_angle = angle_sum;

    printf("避障状态：%d\n",flag_avoid);
}