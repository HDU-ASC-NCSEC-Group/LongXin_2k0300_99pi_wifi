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
********************************************************************************************************************/

#include "zf_common_headfile.h"

// *************************** 例程硬件连接说明 ***************************
//  久久派与主板使用54pin排线连接，再将久久派插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  久久派与主板使用54pin排线连接，再将久久派插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  久久派与主板使用54pin排线连接，再将久久派插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  使用本历程，就需要使用我们逐飞科技提供的内核。
// 
//  目前仅支持两寸SPI屏幕
//  SPI 两寸屏 硬件引脚
//  SCL         查看 seekfree_smart_cat_pai_99 文件中 st7789v 节点定义 默认 GPIO60
//  SDA         查看 seekfree_smart_cat_pai_99 文件中 st7789v 节点定义 默认 GPIO62
//  RST         查看 seekfree_smart_cat_pai_99 文件中 st7789v 节点定义 默认 GPIO74
//  DC          查看 seekfree_smart_cat_pai_99 文件中 st7789v 节点定义 默认 GPIO26
//  CS          查看 seekfree_smart_cat_pai_99 文件中 st7789v 节点定义 默认 GPIO62
//  BL          查看 seekfree_smart_cat_pai_99 文件中 st7789v 节点定义 默认 GPIO63
//  GND         核心板电源地 GND
//  3V3         核心板 3V3 电源
//
// 使用USB插入UVC摄像头
// 
// *************************** 例程测试说明 ***************************
// 1.核心板烧录本例程 插在主板上 2寸IPS 显示模块插在主板的屏幕接口排座上 请注意引脚对应 不要插错
// 
// 2.电池供电 上电后 2寸IPS 屏幕亮起 显示数字等信息
// 
// 3.判断屏幕为SPI屏幕或者并口屏幕: 查看屏幕背面的PCB丝印，如果带有SPI字样就为SPI屏幕，否则为并口屏幕
//   例程文件夹下有不同类型2寸屏幕的区别示意图 "SPI和并口2寸屏幕区别.png"
// 
// 4.运行例程，既可看到灰度图像显示到屏幕上
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查
//
// **************************** 代码区域 ****************************

/*
    opencv库调用
*/
#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <string>

//ips图像显示外部变量引用
extern uint16 ips200_pencolor;
extern cv::Mat frame_rgay;

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 处理程序退出！！！
    // 这里需要关闭电机，关闭电调等。
}

int main(int, char**) 
{
    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    // 初始化屏幕
    ips200_init("/dev/fb0");

    // 初始化UVC摄像头
    if(uvc_camera_init("/dev/video0") < 0)
    {
        return -1;
    }

    // 创建二维码检测器（只创建一次）
    cv::QRCodeDetector qrDecoder;

    // 定义文字显示区域（位于图像下方）
    const int text_x = 0;
    const int text_y = UVC_HEIGHT + 5;      // Y坐标起始（图像高度+5像素）
    const int text_width = UVC_WIDTH;       // 文字区域宽度（与图像宽相同）
    const int text_height = 16;             // 文字区域高度（单个字符高度）
    const uint16 bg_color = IPS200_DEFAULT_BGCOLOR;  // 背景色（白色）

    while(1)
    {
        // 阻塞式等待，图像刷新
        if(wait_image_refresh() < 0)
        {
            // 摄像头未采集到图像，这里需要关闭电机，关闭电调等。
            exit(0);
        }

        // 显示图像到屏幕上（左上角）
        ips200_show_gray_image(0, 0, rgay_image, UVC_WIDTH, UVC_HEIGHT);

        // ---------- 二维码识别 ----------
        std::string qr_data = qrDecoder.detectAndDecode(frame_rgay);

        // ---------- 清除旧的文字区域 ----------
        // 用背景色填充文字区域
        for (uint16 i = 0; i < text_height; i++) {
            for (uint16 j = 0; j < text_width; j++) {
                ips200_draw_point(text_x + j, text_y + i, bg_color);
            }
        }

        // ---------- 显示二维码结果 ----------
        // 设置文字颜色为黑色（背景已为白色）
        uint16 old_pen = ips200_pencolor;  // 注意：现在可以直接访问，因为添加了设置函数，但这里仍会报错？
        // 更好的方式是使用设置函数，而不是直接访问变量。
        // 由于我们添加了设置函数，但未将变量导出，所以不能直接访问 old_pen。
        // 我们只需要设置新颜色，不需要保存旧值（因为其他显示未使用颜色）。
        // 因此直接设置颜色即可。
        ips200_set_pen_color(RGB565_BLACK);
        ips200_set_bg_color(bg_color);

        if (!qr_data.empty()) {
            char buf[64];
            snprintf(buf, sizeof(buf), "QR: %.40s", qr_data.c_str());
            ips200_show_string(text_x, text_y, buf);
        } else {
            ips200_show_string(text_x, text_y, "No QR code");
        }
        
        // 可选：恢复默认颜色（如果不影响其他显示可以不恢复）
        // ips200_set_pen_color(RGB565_RED);
        // ips200_set_bg_color(IPS200_DEFAULT_BGCOLOR);
    }
}

