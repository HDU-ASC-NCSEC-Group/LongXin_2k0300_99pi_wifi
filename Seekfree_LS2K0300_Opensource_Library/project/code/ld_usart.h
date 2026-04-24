#ifndef __ld_usart_h
#define __ld_usart_h

#include "zf_common_headfile.h"

// 乐动D300 STL19雷达数据包格式
#define POINT_PER_PACK   12
#define HEADER  0x54
typedef struct __attribute__((packed)){
    uint16 distance;
    uint8  intensity;
} LidarPointStructDef;
typedef struct __attribute__((packed)){
    uint8 header;
    uint8 ver_len;
    uint16 speed;
    uint16 start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16 end_angle;
    uint16 timestamp;
    uint8 crc8;
}LiDARFrameTypeDef;

#endif