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

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

uint32_t i = 0;

int main(int, char**) 
{
    // IPS200 初始化
    ips200_init("/dev/fb0");
    ips200_clear();  // 清屏为默认背景色（白色）
//    ips200_full(RGB565_BLACK);    // 填充黑色

    ips200_show_string(0, 0, "Hello IPS200");  // 在(0,0)显示字符串
    ips200_show_string(0,16, "Key0:");
    ips200_show_string(0,32, "Key1:");
    ips200_show_string(0,48, "Key2:");
    ips200_show_string(0,64, "Key3:");
    ips200_show_string(0,80, "Switch0:");
    ips200_show_string(0,96, "Switch1:");

    while(1)
    {
        i ++;
        ips200_show_int(40, 16, gpio_get_level(KEY_0), 1);
        ips200_show_int(40, 32, gpio_get_level(KEY_1), 1);
        ips200_show_int(40, 48, gpio_get_level(KEY_2), 1);
        ips200_show_int(40, 64, gpio_get_level(KEY_3), 1);
        ips200_show_int(64, 80, gpio_get_level(SWITCH_0), 1);
        ips200_show_int(64, 96, gpio_get_level(SWITCH_1), 1);
        ips200_show_int(0, 112, i, 8);

        system_delay_ms(100);
    }
}