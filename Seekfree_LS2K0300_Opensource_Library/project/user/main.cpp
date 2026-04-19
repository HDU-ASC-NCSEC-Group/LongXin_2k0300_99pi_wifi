#include "zf_common_headfile.h"
#include "radar_parser.h"
#include "map.h"
#include "laser_point.h"
#include <vector>
#include <cstdio>

int main() {
    // 初始化屏幕
    ips200_init("/dev/fb0");

    // 初始化地图：2m范围，5cm分辨率，小车(120,160)，半径100px，雷达0°朝上(偏移90°)
    map_init(2.0f, 0.05f, 120, 160, 100, 90.0f);

    // 初始化雷达（使用您可用的 RadarParser）
    RadarParser radar;
    if (!radar.begin("/dev/ttyUSB0", 230400)) {
        printf("Radar open failed!\n");
        return -1;
    }

    RadarPacket packet;
    std::vector<LaserPoint> scan_buffer;
    float last_angle = 0.0f;
    bool first_packet = true;

    while (1) {
        if (radar.readPacket(packet)) {
            // 将12个点插入缓冲区
            for (int i = 0; i < 12; ++i) {
                uint16_t dist = packet.points[i].distance;
                if (dist == 0) continue;

                // 角度插值
                float angle = packet.start_angle + 
                              (packet.end_angle - packet.start_angle) * i / 11.0f;

                LaserPoint pt;
                pt.angle_deg = angle;
                pt.distance_mm = dist;
                pt.intensity = packet.points[i].intensity;
                scan_buffer.push_back(pt);
            }

            // 一圈结束判断：起始角度变小（越过360°→0°）
            if (!first_packet && packet.start_angle < last_angle) {
                map_update(scan_buffer.data(), scan_buffer.size());
                scan_buffer.clear();
            }

            last_angle = packet.start_angle;
            first_packet = false;
        } else {
            // 读包失败时重绘（避免黑屏）
            map_redraw();
        }

        usleep(5000);
    }

    radar.end();
    return 0;
}