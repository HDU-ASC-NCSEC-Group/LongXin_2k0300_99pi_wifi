#include "zf_common_headfile.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#define BEEP "/dev/zf_driver_gpio_beep"

//ips图像显示外部变量引用
extern uint16 ips200_pencolor;
extern cv::Mat frame_rgay;

//二维码相关数据初始化
// 创建二维码检测器（只创建一次）
cv::QRCodeDetector qrDecoder;

// 定义文字显示区域（位于图像下方）
const int text_x = 0;
const int text_y = UVC_HEIGHT + 5;      // Y坐标起始（图像高度+5像素）
const int text_width = UVC_WIDTH;       // 文字区域宽度（与图像宽相同）
const int text_height = 16;             // 文字区域高度（单个字符高度）
const uint16 bg_color = IPS200_DEFAULT_BGCOLOR;  // 背景色（白色）


// 二维码解码处理
void QR_process(void)
{
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
            
        //检测到二维码，蜂鸣器响
        gpio_set_level(BEEP, 0x1);
    } else {
        ips200_show_string(text_x, text_y, "No QR code");
        //未检测到二维码，蜂鸣器关
        gpio_set_level(BEEP, 0x0);
    }
        
    // 可选：恢复默认颜色（如果不影响其他显示可以不恢复）
    // ips200_set_pen_color(RGB565_RED);
    // ips200_set_bg_color(IPS200_DEFAULT_BGCOLOR);
}

void object_tracking(void)
{
    
}
