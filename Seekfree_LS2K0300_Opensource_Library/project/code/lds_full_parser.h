// lds_full_parser.h
#ifndef LDS_FULL_PARSER_H
#define LDS_FULL_PARSER_H
#include <vector>
#include <cstdint>
#include "laser_point.h"

class LDSFullParser {
public:
    bool begin(const char* port, int baud);
    void end();
    bool grabFullScan(std::vector<LaserPoint>& out, uint32_t timeout_ms = 2000);
private:
    int fd_;
    std::vector<LaserPoint> scan_buffer_;
    float last_angle_ = 0.0f;
    bool readBytes(uint8_t* buf, size_t len);
    bool parsePacket(const uint8_t* data, size_t len);
};
#endif