#include "radar_parser.h"
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cerrno>

RadarParser::RadarParser() : fd_(-1) {}

RadarParser::~RadarParser() {
    end();
}

bool RadarParser::begin(const char* port, int baudrate) {
    // 打开串口
    fd_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        return false;
    }

    // 设置为阻塞模式
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NDELAY);

    struct termios options;
    tcgetattr(fd_, &options);

    // 设置波特率
    speed_t baud_const;
    switch (baudrate) {
        case 230400: baud_const = B230400; break;
        case 115200: baud_const = B115200; break;
        case 9600:   baud_const = B9600;   break;
        default:     baud_const = B230400; break;
    }
    cfsetispeed(&options, baud_const);
    cfsetospeed(&options, baud_const);

    // 控制模式：8N1，本地，启用接收
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;          // 无硬件流控

    // 本地模式：原始输入，无回显
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 输入模式：不做任何转换
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    // 输出模式：原始输出
    options.c_oflag &= ~OPOST;

    // 读取超时：0.5秒
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;

    tcsetattr(fd_, TCSANOW, &options);
    tcflush(fd_, TCIOFLUSH);

    return true;
}

void RadarParser::end() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

ssize_t RadarParser::readBytes(uint8_t* buffer, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t ret = read(fd_, buffer + total, len - total);
        if (ret <= 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return -1;
        }
        total += ret;
    }
    return total;
}

bool RadarParser::parseBuffer(const uint8_t* buffer, RadarPacket& packet) {
    // 检查包头
    if (buffer[0] != HEADER1 || buffer[1] != HEADER2) {
        return false;
    }

    // 转速 (小端序)
    packet.speed = buffer[2] | (buffer[3] << 8);

    // 起始角度 (0.01度单位)
    uint16_t start_raw = buffer[4] | (buffer[5] << 8);
    packet.start_angle = start_raw * 0.01f;

    // 12个测量点
    int offset = 6;
    for (int i = 0; i < 12; ++i) {
        packet.points[i].distance = buffer[offset] | (buffer[offset + 1] << 8);
        packet.points[i].intensity = buffer[offset + 2];
        offset += 3;
    }

    // 结束角度
    uint16_t end_raw = buffer[offset] | (buffer[offset + 1] << 8);
    packet.end_angle = end_raw * 0.01f;
    offset += 2;

    // 时间戳
    packet.timestamp = buffer[offset] | (buffer[offset + 1] << 8);
    offset += 2;

    // CRC
    packet.crc = buffer[offset];

    return true;
}

bool RadarParser::readPacket(RadarPacket& packet) {
    if (fd_ < 0) return false;

    // 同步到包头 0x54 0x2C
    uint8_t sync = 0;
    while (true) {
        if (readBytes(&sync, 1) <= 0) return false;
        if (sync == HEADER1) {
            uint8_t next;
            if (readBytes(&next, 1) <= 0) continue;
            if (next == HEADER2) {
                break;
            }
        }
    }

    // 读取剩余 45 字节
    uint8_t remaining[45];
    if (readBytes(remaining, 45) != 45) {
        return false;
    }

    // 构造完整包
    uint8_t full[47];
    full[0] = HEADER1;
    full[1] = HEADER2;
    std::memcpy(full + 2, remaining, 45);

    return parseBuffer(full, packet);
}
