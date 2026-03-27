#include "zf_driver_uart.h"
#include <string.h>

// 这里假设有底层硬件相关的寄存器和初始化函数
// 你需要根据2k0300芯片实际手册补充底层寄存器操作
// 这里只做伪代码和接口演示

#define UART_BASE_ADDR   (0x40000000) // 示例地址，请替换为实际串口寄存器基址
#define UART_BAUDRATE    9600

// 假设有如下硬件寄存器结构体
// volatile struct UART_TypeDef {
//     volatile uint32_t DATA;
//     volatile uint32_t STATUS;
//     volatile uint32_t BAUD;
//     ...
// } *UART = (UART_TypeDef*)UART_BASE_ADDR;

void zf_uart_init(void)
{
    // TODO: 根据2k0300实际硬件初始化串口，设置波特率9600、8N1
    // UART->BAUD = ...;
    // UART->CTRL = ...;
}

void zf_uart_send_byte(uint8_t data)
{
    // TODO: 等待发送缓冲区空，再写入数据
    // while (!(UART->STATUS & TX_EMPTY)) ;
    // UART->DATA = data;
}

void zf_uart_send_string(const char *str)
{
    while (*str) {
        zf_uart_send_byte((uint8_t)*str++);
    }
}

bool zf_uart_available(void)
{
    // TODO: 检查接收缓冲区是否有数据
    // return (UART->STATUS & RX_NOT_EMPTY) != 0;
    return false;
}

uint16_t zf_uart_receive_line(char *buf, uint16_t max_len)
{
    uint16_t idx = 0;
    while (idx < max_len - 1) {
        // 轮询等待数据
        while (!zf_uart_available()) ;
        // TODO: 读取一个字节
        // char c = UART->DATA;
        char c = '\0'; // 占位
        buf[idx++] = c;
        if (c == '\n') break;
    }
    buf[idx] = '\0';
    return idx;
}
