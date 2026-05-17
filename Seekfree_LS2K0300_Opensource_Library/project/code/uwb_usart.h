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

#ifndef _uwb_usart_h_
#define _uwb_usart_h_

#include "zf_common_typedef.h"

//==================================================UWB 串口配置===================================================

#define UWB_UART_DEVICE         "/dev/ttyUSB0"
#define UWB_UART_BAUDRATE       115200      // 波特率数值（非 Bxxx 常量，用于 termios2 BOTHER 模式）
#define UWB_RX_BUF_SIZE         128
#define UWB_RX_DLY_MAX          50

//==================================================UWB 数据校准====================================================

#define UWB_DIST_SCALE          10.81f       // 距离比例因子 (raw * scale + offset = 校正值)
#define UWB_DIST_OFFSET_MM      16.26f         // 距离补偿偏移 (mm)，正数=增加输出值
#define UWB_FILTER_ALPHA        0.25f       // EMA 滤波系数 (0.0~1.0)，越小越平滑

//==================================================UWB 接收状态宏==================================================

#define UWB_UART_IDLE           0
#define UWB_UART_RECVING        1
#define UWB_UART_RECED          2

//==================================================UWB 命令定义====================================================

#define UWB_CMD_DIST_AZIMUTH    0x2001      // 距离 + 方位角 + 仰角上报

//==================================================UWB 数据帧结构体================================================

typedef struct __attribute__((packed))
{
    uint32 frame_header;        // 帧头 0xFFFFFFFF
    uint16 packet_length;       // 包长度
    uint16 sequence_id;         // 序列号
    uint16 request_command;     // 请求命令字
    uint16 version_id;          // 版本号
    uint32 anchor_id;           // 基站 ID
    uint32 tag_id;              // 标签 ID
    int32  distance;            // 距离 (mm) 大端序
    int16  azimuth;             // 方位角 (0.01°) 大端序
    int16  elevation;           // 仰角 (0.01°) 大端序
    uint16 tag_status;          // 标签状态
    uint16 batch_sn;            // 批次号
    uint32 reserve;             // 保留
    uint8  xor_check;           // 异或校验
} UWB_RawFrame;

//==================================================UWB 解算数据====================================================

typedef struct
{
    int32  distance;            // 原始距离 (mm)，直接来自帧数据
    float  azimuth_deg;         // 原始方位角 (度)
    float  elevation_deg;       // 原始仰角 (度)
    uint32 tag_id;              // 标签 ID
    uint32 anchor_id;           // 基站 ID
    uint8  data_valid;          // 数据有效标志 非零有效
    uint32 timestamp;           // 最后更新时间戳

    float  distance_m;          // 滤波后距离 (米)，已含比例+偏移校准
    float  azimuth_f;           // 滤波后方位角 (度)
    float  elevation_f;         // 滤波后仰角 (度)
} UWB_Data;

//==================================================UWB 接收控制结构体===============================================

typedef struct
{
    uint8  buf[UWB_RX_BUF_SIZE];
    uint8  rx_status;
    uint8  get_check;
    uint8  inc_num;
    uint8  inc_limit;
    uint8  end_flag;
    uint16 command;
    uint16 rx_num;
    uint32 rx_time;
} UWB_RxCtrl;

extern UWB_Data g_uwb_data;
extern volatile uint32_t g_uwb_frame_count;    // 每收到一帧 +1，供外部检测数据刷新

//==================================================API 函数声明====================================================

void uwb_usart_init        (const char *device, uint32 baudrate);
void uwb_usart_task        (void);
void uwb_usart_close       (void);

uint16 uwb_byte_swap16     (uint16 val);
uint32 uwb_byte_swap32     (uint32 val);

#endif
