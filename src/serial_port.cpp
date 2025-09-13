// src/serial_port.cpp
#include "serial_port.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

SerialPort::SerialPort(const std::string &port_name, uint32_t baud_rate)
    : fd_(-1),                  // 1. 对应声明顺序第1个
      port_name_(port_name),    // 2. 对应声明顺序第2个
      baud_rate_(baud_rate),    // 3. 对应声明顺序第3个
      is_open_(false)           // 4. 对应声明顺序第4个
{}

SerialPort::~SerialPort()
{
    close();
}

speed_t SerialPort::baudRateToSpeed(uint32_t baud_rate)
{
    switch (baud_rate)
    {
    case 9600:
        return B9600;
    case 115200:
        return B115200;
    case 57600:
        return B57600;
    case 38400:
        return B38400;
    default:
        return B9600; // 默认9600波特率
    }
}

bool SerialPort::open()
{
    // 打开串口设备（读写模式，不阻塞->阻塞模式）
    // fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY); // 去掉O_NONBLOCK
    if (fd_ == -1)
    {
        std::cerr << "Failed to open serial port: " << port_name_ << std::endl;
        return false;
    }

    // 配置串口参数
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0)
    {
        std::cerr << "Failed to get serial port attributes" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 设置波特率
    speed_t speed = baudRateToSpeed(baud_rate_);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 配置数据格式：8位数据位，无校验位，1位停止位
    tty.c_cflag &= ~PARENB;        // 无校验
    tty.c_cflag &= ~CSTOPB;        // 1位停止位
    tty.c_cflag &= ~CSIZE;         // 清除数据位设置
    tty.c_cflag |= CS8;            // 8位数据位
    tty.c_cflag |= CREAD | CLOCAL; // 启用接收，忽略调制解调器控制线

    // 应用配置
    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "Failed to set serial port attributes" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    is_open_ = true;
    std::cout << "Serial port " << port_name_ << " opened successfully" << std::endl;
    return true;
}

void SerialPort::close()
{
    if (is_open_)
    {
        ::close(fd_);
        is_open_ = false;
        fd_ = -1;
        std::cout << "Serial port " << port_name_ << " closed" << std::endl;
    }
}

bool SerialPort::isOpen() const
{
    return is_open_;
}

bool SerialPort::send(const std::string &data)
{
    if (!is_open_)
    {
        std::cerr << "Serial port not open" << std::endl;
        return false;
    }

    // 直接发送传入的数据（不额外添加换行符，因为main.cpp中已正确添加）
    ssize_t bytes_written = ::write(fd_, data.c_str(), data.size());
    if (bytes_written != static_cast<ssize_t>(data.size()))
    {
        std::cerr << "Failed to send data: " << data << std::endl;
        return false;
    }

    return true;
}