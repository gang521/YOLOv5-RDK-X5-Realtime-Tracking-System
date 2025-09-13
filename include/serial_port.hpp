// include/serial_port.hpp
#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <string>
#include <cstdint>
#include <termios.h>

class SerialPort {
public:
    SerialPort(const std::string& port_name, uint32_t baud_rate);
    ~SerialPort();

    bool open();  // 打开串口
    void close(); // 关闭串口
    bool isOpen() const; // 检查串口是否打开
    bool send(const std::string& data); // 发送数据
    
    // 公开文件描述符以便读取操作
    int fd_; // 串口文件描述符

private:
    std::string port_name_;
    uint32_t baud_rate_;
    bool is_open_;

    // 波特率转换（Linux系统需要）
    speed_t baudRateToSpeed(uint32_t baud_rate);
};

#endif // SERIAL_PORT_HPP
