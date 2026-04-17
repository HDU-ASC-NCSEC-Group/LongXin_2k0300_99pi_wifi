
/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-02-27        大W            first version
* 2026-03-22        AI             添加红色物块识别与坐标显示
********************************************************************************************************************/
#include "zf_common_headfile.h"

extern struct pwm_info servo_pwm_info;

#define RAD_TO_DEG (180.0f / 3.14159265358979f)

timer_fd *pit_timer;

float roll, pitch, yaw;

// 屏幕显示区域定义（基于320x240分辨率）
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

// 布局坐标
#define TITLE_X        10
#define TITLE_Y        10

#define SPEED_X        10
#define SPEED_Y        30

#define ANGLE_X        10
#define ANGLE_Y        50

#define TIMESTAMP_X    10
#define TIMESTAMP_Y    70

#define POINTS_START_X 10
#define POINTS_START_Y 90
#define POINTS_COL_GAP 75    // 每列宽度
#define POINTS_ROW_GAP 12    // 每行高度
#define POINTS_PER_COL 6     // 每列显示6个点

// 里面放入中断代码
void pit_callback()
{
    
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 此处可添加电机、电调等关闭代码
}

int main(int, char**) 
{
    // 注册清理函数和信号处理
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    // 初始化屏幕
    ips200_init("/dev/fb0");

    // 四元数初始化
    // quaternion_init(0.5f, 0.05f);  // 设置适当的增益


    // 初始化UVC摄像头
    // if (uvc_camera_init("/dev/video0") < 0) {
    //     return -1;
    // }

    //获取PWM设备信息
    pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);

    // 创建一个定时器1ms周期，回调函数为pit_callback
    pit_timer = new timer_fd(1, pit_callback);
    pit_timer->start();

    // 激光雷达解析器初始化
    RadarParser radar;
    if (!radar.begin("/dev/ttyUSB0", 230400)) {
        ips200_set_pen_color(RGB565_RED);
        ips200_show_string(10, 100, "Radar init failed!");
    }

    RadarPacket packet;

    // 主循环
    while (1) {
        // object_tracking();  // 红色物块跟踪显示
        // coordinate_transformation();  // 坐标转换显示

        if (radar.readPacket(packet)) {
            // 清空数据显示区域（可选，可优化为局部清除）
            ips200_full(RGB565_WHITE);

            // 显示标题
            ips200_show_string(TITLE_X, TITLE_Y, "LDR D300 Radar");

            // 显示转速
            ips200_Printf(SPEED_X, SPEED_Y, "Speed: %d dps", packet.speed);

            // 显示角度范围
            ips200_Printf(ANGLE_X, ANGLE_Y, "Angle: %.2f -> %.2f", packet.start_angle, packet.end_angle);

            // 显示时间戳
            ips200_Printf(TIMESTAMP_X, TIMESTAMP_Y, "Time: %u ms", packet.timestamp);

            // 显示测量点（分两列显示，每列6个点）
            ips200_show_string(POINTS_START_X, POINTS_START_Y - 12, "Points (dist/mm, int):");

            for (int i = 0; i < 12; ++i) {
                int col = i / POINTS_PER_COL;
                int row = i % POINTS_PER_COL;
                int x = POINTS_START_X + col * POINTS_COL_GAP;
                int y = POINTS_START_Y + row * POINTS_ROW_GAP;

                ips200_Printf(x, y, "%2d: %5d %3d", i, 
                              packet.points[i].distance, 
                              packet.points[i].intensity);
            }

            // 显示CRC（可选）
            ips200_Printf(10, 220, "CRC: 0x%02X", packet.crc);
        }

    }

    return 0;
}
