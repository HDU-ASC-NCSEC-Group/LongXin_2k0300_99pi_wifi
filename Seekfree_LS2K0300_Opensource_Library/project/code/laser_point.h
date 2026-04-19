#ifndef LASER_POINT_H
#define LASER_POINT_H

#include <cstdint>

struct LaserPoint {
    float    angle_deg;    // 角度（度）
    uint16_t distance_mm;  // 距离（毫米）
    uint8_t  intensity;    // 信号强度（0-255）
};

#endif