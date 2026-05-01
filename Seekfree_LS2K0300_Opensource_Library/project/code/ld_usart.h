#ifndef __ld_usart_h
#define __ld_usart_h

#include "zf_common_headfile.h"
#include <stdint.h>

#define POINT_PER_PACK   12
#define PACKET_SIZE      47        // 1+1+2+2+12*3+2+2+1

#pragma pack(1)
typedef struct {
    uint16_t distance;
    uint8_t  intensity;
} LidarPointStructDef;

typedef struct {
    float    angle;      // 平均角度（度）
    uint16_t distance;   // 平均距离（mm）
} PointData_t;

typedef struct {
    uint8_t  header;
    uint8_t  ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t  crc8;
} LiDARFrameTypeDef;
#pragma pack()

extern LiDARFrameTypeDef g_lidar_frame;
extern volatile bool g_lidar_frame_valid;
extern PointData_t PointDataProcess[50]; // 环形缓冲区，存储50个点的数据

bool ld_usart_init(const char *device, int baudrate);
void ld_usart_task(void);
void data_process(void);

#endif