#include "serial_port.hpp"
#include <thread>
#include <iostream>  // 必须包含这个头文件才能使用 std::cout

int main() {
    SerialPort serial("/dev/ttyACM0", 9600);
    if (serial.open()) {
        std::cout << "串口打开，延迟10秒后关闭（不发送任何数据）" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));  // 延迟10秒
        serial.close();
    } else {
        std::cerr << "串口打开失败" << std::endl;
    }
    return 0;
}