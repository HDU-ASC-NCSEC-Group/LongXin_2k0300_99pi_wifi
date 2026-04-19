#ifndef MAP_H
#define MAP_H

#include <cstdint>
#include <cstddef>
#include "laser_point.h"

#ifdef __cplusplus
extern "C" {
#endif

void map_init(float map_size_m, float grid_res_m,
              uint16_t car_screen_x, uint16_t car_screen_y,
              uint8_t screen_radius_px, float angle_offset_deg);

void map_update(const LaserPoint* points, size_t count);
void map_clear();
void map_redraw();

#ifdef __cplusplus
}
#endif

#endif // MAP_H