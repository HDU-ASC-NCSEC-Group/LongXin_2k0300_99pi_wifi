#ifndef _ZF_DRIVER_UART_H_
#define _ZF_DRIVER_UART_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 串口初始化，波特率9600，8N1
void zf_uart_init(void);

// 发送字符串（以\0结尾）
void zf_uart_send_string(const char *str);

// 发送单个字节
void zf_uart_send_byte(uint8_t data);

// 接收一行（以\n为结束），返回接收字节数，buf自动加\0结尾
// max_len为buf最大长度，建议>=64
uint16_t zf_uart_receive_line(char *buf, uint16_t max_len);

// 查询是否有数据可读
bool zf_uart_available(void);

#ifdef __cplusplus
}
#endif

#endif // _ZF_DRIVER_UART_H_
