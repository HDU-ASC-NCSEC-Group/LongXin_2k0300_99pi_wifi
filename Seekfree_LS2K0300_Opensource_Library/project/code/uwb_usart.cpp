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
* 文件名称          uwb_usart
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-02-27        大W            first version
********************************************************************************************************************/

#include "uwb_usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>

//==================================================内部数据定义====================================================

static int              g_uwb_fd = -1;
static UWB_RxCtrl       g_uwb_rx;
UWB_Data                g_uwb_data;

typedef union
{
    uint8          raw[sizeof(UWB_RawFrame)];
    UWB_RawFrame   item;
} UWB_RawBuf;

static UWB_RawBuf       g_uwb_buf;

//==================================================内部函数声明====================================================

static void process_byte   (uint8 byte);
static void parse_frame    (void);
static void reset_rx_ctrl  (void);
static int  ser_available  (void);

//==================================================字节序转换======================================================
// UWB 模块数据为大端序，LS2K0300 为小端序

uint16 uwb_byte_swap16(uint16 val)
{
    return (val >> 8) | (val << 8);
}

uint32 uwb_byte_swap32(uint32 val)
{
    return ((val & 0x000000FF) << 24) |
           ((val & 0x0000FF00) <<  8) |
           ((val & 0x00FF0000) >>  8) |
           ((val & 0xFF000000) >> 24);
}

//==================================================内部函数========================================================

static void reset_rx_ctrl(void)
{
    g_uwb_rx.rx_status   = UWB_UART_IDLE;
    g_uwb_rx.rx_time     = 0;
    g_uwb_rx.rx_num      = 0;
    g_uwb_rx.get_check   = 0;
    g_uwb_rx.inc_num     = 0;
    g_uwb_rx.inc_limit   = 0;
    g_uwb_rx.command     = 0;
    g_uwb_rx.end_flag    = 0;
}

static int ser_available(void)
{
    if (g_uwb_fd < 0) return 0;

    fd_set rfds;
    struct timeval tv = {0, 0};

    FD_ZERO(&rfds);
    FD_SET(g_uwb_fd, &rfds);

    return select(g_uwb_fd + 1, &rfds, NULL, NULL, &tv) > 0;
}

static void process_byte(uint8 byte)
{
    if ((g_uwb_rx.rx_status != UWB_UART_IDLE) &&
        (g_uwb_rx.rx_status != UWB_UART_RECVING))
        return;

    // 写入接收缓冲区
    g_uwb_buf.raw[g_uwb_rx.rx_num++] = byte;
    g_uwb_rx.rx_time = UWB_RX_DLY_MAX;
    g_uwb_rx.rx_status = UWB_UART_RECVING;

    // 帧头检测：收到 0xFF 时递增，否则清零
    if (byte == 0xFF)
        g_uwb_rx.end_flag++;
    else
        g_uwb_rx.end_flag = 0;

    // 连续 4 个 0xFF → 检测到帧头 (含重同步)
    if (g_uwb_rx.end_flag >= 4)
    {
        g_uwb_rx.end_flag  = 0;
        g_uwb_rx.get_check = 1;
        g_uwb_rx.inc_num   = 0;
        g_uwb_rx.inc_limit = 0;
        g_uwb_rx.command   = 0;
        g_uwb_rx.rx_num    = 4;
        return;
    }

    // 帧头已锁定，逐字节统计并识别命令
    if (g_uwb_rx.get_check)
    {
        g_uwb_rx.inc_num++;

        if (g_uwb_rx.inc_num == 5)
        {
            g_uwb_rx.command = byte;       // 命令字高字节
        }

        if (g_uwb_rx.inc_num == 6)
        {
            g_uwb_rx.command = (g_uwb_rx.command << 8) | byte; // 命令字低字节

            if (g_uwb_rx.command == UWB_CMD_DIST_AZIMUTH)
            {
                g_uwb_rx.inc_limit = 33;   // 此命令固定 33 字节载荷 (总帧长 37)
            }
        }

        if (g_uwb_rx.inc_num == g_uwb_rx.inc_limit)
        {
            g_uwb_rx.rx_status = UWB_UART_RECED;
        }
    }

    // 缓冲区溢出保护
    if (g_uwb_rx.rx_num >= UWB_RX_BUF_SIZE)
    {
        g_uwb_rx.rx_status = UWB_UART_RECED;
    }
}

static void parse_frame(void)
{
    if (g_uwb_rx.command != UWB_CMD_DIST_AZIMUTH)
        return;

    // 校验帧长 (37 bytes)
    if (g_uwb_rx.rx_num < 37)
        return;

    // 提取并转换关键数据（大端序 → 小端序）
    int32  dist   = (int32)uwb_byte_swap32(g_uwb_buf.item.distance);
    int16  azim   = (int16)uwb_byte_swap16((uint16)g_uwb_buf.item.azimuth);
    int16  elev   = (int16)uwb_byte_swap16((uint16)g_uwb_buf.item.elevation);
    uint32 tag_id = uwb_byte_swap32(g_uwb_buf.item.tag_id);
    uint32 anc_id = uwb_byte_swap32(g_uwb_buf.item.anchor_id);

    g_uwb_data.distance      = dist;
    g_uwb_data.azimuth_deg   = (float)azim * 0.01f;
    g_uwb_data.elevation_deg = (float)elev * 0.01f;
    g_uwb_data.tag_id        = tag_id;
    g_uwb_data.anchor_id     = anc_id;
    g_uwb_data.data_valid    = 1;
    g_uwb_data.timestamp     = 0;   // 由调用方按需填入 tick
}

//==================================================外部 API=========================================================

void uwb_usart_init(const char *device, uint32 baudrate)
{
    if (g_uwb_fd >= 0)
        uwb_usart_close();

    reset_rx_ctrl();
    memset(&g_uwb_data, 0, sizeof(g_uwb_data));
    memset(&g_uwb_buf,  0, sizeof(g_uwb_buf));

    g_uwb_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_uwb_fd < 0)
    {
        printf("[UWB] open %s failed: %s\r\n", device, strerror(errno));
        return;
    }

    struct termios opt;
    tcgetattr(g_uwb_fd, &opt);

    cfsetispeed(&opt, baudrate);
    cfsetospeed(&opt, baudrate);

    opt.c_cflag |=  (CLOCAL | CREAD);
    opt.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    opt.c_cflag |=  CS8;
    opt.c_cflag &= ~CRTSCTS;

    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~OPOST;
    opt.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);

    opt.c_cc[VMIN]  = 0;
    opt.c_cc[VTIME] = 0;

    tcsetattr(g_uwb_fd, TCSANOW, &opt);
    tcflush(g_uwb_fd, TCIOFLUSH);

    // 设为非阻塞
    int flags = fcntl(g_uwb_fd, F_GETFL, 0);
    fcntl(g_uwb_fd, F_SETFL, flags | O_NONBLOCK);

    printf("[UWB] init ok, dev=%s baud=%u\r\n", device, baudrate);
}

void uwb_usart_task(void)
{
    if (g_uwb_fd < 0) return;

    // 超时检测
    if (g_uwb_rx.rx_time)
    {
        g_uwb_rx.rx_time--;
        if (g_uwb_rx.rx_time == 0)
            g_uwb_rx.rx_status = UWB_UART_RECED;
    }

    // 快速拉取所有可用字节
    while (ser_available())
    {
        uint8 byte;
        int n = read(g_uwb_fd, &byte, 1);
        if (n <= 0) break;
        process_byte(byte);
    }

    // 帧接收完毕 → 解算
    if (g_uwb_rx.rx_status == UWB_UART_RECED)
    {
        parse_frame();
        reset_rx_ctrl();

        // 打印解析结果
        if (g_uwb_data.data_valid)
        {
            printf("Dist=%.2fm Azi=%.2fdeg Elev=%.2fdeg\r\n",
                   (float)g_uwb_data.distance * 0.001f,
                   g_uwb_data.azimuth_deg,
                   g_uwb_data.elevation_deg);
            g_uwb_data.data_valid = 0;
        }
    }
}

void uwb_usart_close(void)
{
    if (g_uwb_fd >= 0)
    {
        close(g_uwb_fd);
        g_uwb_fd = -1;
        printf("[UWB] closed\r\n");
    }
}
