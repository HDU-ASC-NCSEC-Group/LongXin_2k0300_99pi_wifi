#ifndef RADAR_PARSER_H
#define RADAR_PARSER_H

#include <cstdint>
#include <vector>
#include <unistd.h>

// 雷达数据包结构体
struct RadarPacket {
    uint16_t speed;                     // 转速 (度/秒)
    float start_angle;                  // 起始角度 (度)
    float end_angle;                    // 结束角度 (度)
    uint16_t timestamp;                 // 时间戳 (ms)
    uint8_t crc;                        // CRC校验值
    struct {
        uint16_t distance;              // 距离 (mm)
        uint8_t intensity;              // 强度
    } points[12];                       // 12个测量点
};

class RadarParser {
public:
    RadarParser();
    ~RadarParser();

    // 初始化串口，返回是否成功
    bool begin(const char* port = "/dev/ttyUSB0", int baudrate = 230400);

    // 读取并解析一个数据包，返回是否成功
    bool readPacket(RadarPacket& packet);

    // 关闭串口
    void end();

private:
    int fd_;                            // 串口文件描述符
    static constexpr int PACKET_SIZE = 47;
    static constexpr uint8_t HEADER1 = 0x54;
    static constexpr uint8_t HEADER2 = 0x2C;

    // 从串口读取指定字节数，返回实际读取字节数
    ssize_t readBytes(uint8_t* buffer, size_t len);

    // 解析原始字节流到 packet
    bool parseBuffer(const uint8_t* buffer, RadarPacket& packet);
};

#endif // RADAR_PARSER_H
