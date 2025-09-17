#include "camera_yolo.hpp"
#include "yolov5_detector.hpp"
#include "serial_port.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <thread>
#include <unistd.h>

#define MODEL_PATH "../models/yolov5n_new_nv12.bin"
#define SERIAL_PORT "/dev/ttyACM0"
#define BAUD_RATE 9600
#define SEND_INTERVAL 0.05    // 发送间隔
#define DETECT_SKIP_FRAMES 4 // 每隔4帧检测一次
#define SERIAL_DELAY_MS 2000 // 串口初始化后有一个延迟
#define NETWORK_INTERFACE "eth0"  // 网络接口名
#define OUTBOUND_THRESHOLD 10

// 程序状态枚举
enum class ProgramState {
    WAITING_FOR_START,  // 等待Arduino发送'1'开始
    DETECTING,          // 正在检测
};

// 断网运行的函数封装（在sudo下）
void disableNetwork() {
    std::string cmd = "sudo ip link set " + std::string(NETWORK_INTERFACE) + " down";
    int result = system(cmd.c_str());
    
    if (result == 0) {
        std::cout << "成功断开网络接口: " << NETWORK_INTERFACE << std::endl;
    } else {
        std::cerr << "警告: 断网命令执行失败（可能需要root权限）" << std::endl;
    }
}

int main(int argc, char **argv)
{
    disableNetwork();

    int outOfBoxCounter = 0;

    bool show_display = false;
    if (argc > 1 && std::string(argv[1]) == "--display")
    {
        show_display = true;
    }

    ProgramState current_state = ProgramState::WAITING_FOR_START;

    // 初始化串口
    SerialPort serial(SERIAL_PORT, BAUD_RATE);
    bool serial_available = false;
    if (serial.open())
    {
        std::cout << "串口初始化成功，等待2秒稳定..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(SERIAL_DELAY_MS));
        serial_available = true;
    }
    else
    {
        std::cerr << "警告：无法打开串口 " << SERIAL_PORT
                  << "，将继续运行但不发送数据" << std::endl;
    }

    // 等待开始信号
    std::cout << "等待开始信号（串口发送'1'）..." << std::endl;
    if (serial_available)
    {
        while (true)
        {
            char buf[1];
            int n = ::read(serial.fd_, buf, 1);
            if (n > 0 && buf[0] == '1')
            {
                std::cout << "收到开始信号，即将启动检测..." << std::endl;
                current_state = ProgramState::DETECTING;
                std::cout << "[状态切换] 已从WAITING_FOR_START切换为DETECTING" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else
    {
        std::cerr << "串口不可用，跳过等待信号，直接启动检测..." << std::endl;
        current_state = ProgramState::DETECTING;
    }
    
    // 初始化相机
    HikCamera camera;
    if (!camera.connect())
    {
        std::cerr << "Failed to connect to camera" << std::endl;
        return -1;
    }

    // 初始化YOLOv5检测器
    YOLOv5Detector detector(MODEL_PATH);
    if (!detector.initialize())
    {
        std::cerr << "Failed to initialize YOLOv5 detector" << std::endl;
        return -1;
    }

    // 时间与计数变量
    auto last_send_time = std::chrono::system_clock::now();
    auto prev_time = std::chrono::system_clock::now();
    int frame_count = 0;

    // 胜利条件变量
    auto center_in_box_start_time = std::chrono::system_clock::now();
    bool center_in_box = false;
    const unsigned long long VICTORY_DURATION = 2500000000; // 毫秒
    bool victory_signal_sent = false;

    cv::Mat frame;

    // 帧率测量变量
    auto camera_start_time = std::chrono::system_clock::now();
    int camera_frame_count = 0;

    // 无限循环
    while (true)
    {
        frame_count++;

        // 读取相机帧
        if (!camera.read(frame))
        {
            std::cerr << "Failed to read frame from camera" << std::endl;
            break;
        }

        // 测量相机帧率
        camera_frame_count++;
        auto current_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = current_time - camera_start_time;
        
        if (elapsed.count() >= 1.0)
        {
            double camera_fps = camera_frame_count / elapsed.count();
            std::cout << "[相机帧率] " << std::fixed << std::setprecision(1) << camera_fps << " 帧/秒" << std::endl;
            camera_frame_count = 0;
            camera_start_time = current_time;
        }

        // 每5帧检测一次
        bool is_detection_frame = (frame_count % (DETECT_SKIP_FRAMES + 1) == 0);
        std::vector<cv::Point2i> centers;
        cv::Rect2d best_bbox;
        bool has_high_confidence_detection = false;

        if (is_detection_frame)
        {
            // 执行检测
            auto start_inf = std::chrono::system_clock::now();
            auto results = detector.detect(frame, true);
            auto end_inf = std::chrono::system_clock::now();
            std::chrono::duration<double, std::milli> inf_time = end_inf - start_inf;
            std::cout << "[推理时间] " << std::fixed << std::setprecision(2) << inf_time.count() << " 毫秒" << std::endl;

            // 计算FPS
            current_time = std::chrono::system_clock::now();
            std::chrono::duration<double> fps_duration = current_time - prev_time;
            prev_time = current_time;

            // 提取面积最大的检测框
            if (!results.empty())
            {
                // 改为按检测框面积（宽×高）筛选最大的框
                auto max_result_it = std::max_element(results.begin(), results.end(),
                    [](const YOLOv5Detector::DetectionResult &a, const YOLOv5Detector::DetectionResult &b) {
                        float area_a = a.bbox.width * a.bbox.height; // 检测框a的面积
                        float area_b = b.bbox.width * b.bbox.height; // 检测框b的面积
                        return area_a < area_b; // 面积小的排在前面
                    });

                // 输出最大框的面积和置信度（调试用）
                float max_area = max_result_it->bbox.width * max_result_it->bbox.height;
                std::cout << "[最大检测框] 面积: " << max_area 
                          << ", 置信度: " << std::fixed << std::setprecision(4) << max_result_it->score << std::endl;

                // 仍可保留置信度筛选（可选，避免选取极低置信度的大框）
                if (max_result_it->score > 0.4f) { 
                    int cx = static_cast<int>(max_result_it->center.x);
                    int cy = static_cast<int>(max_result_it->center.y);
                    centers.emplace_back(cx, cy);
                    best_bbox = max_result_it->bbox;
                    has_high_confidence_detection = true;
                }
            }

            // 胜利条件判断
            if (current_state == ProgramState::DETECTING) {
                cv::Point2f target_point(640.0f, 520.0f);
                bool now_in_box = has_high_confidence_detection && best_bbox.contains(target_point);

                std::cout << "[判断调试] 基准点: (" << target_point.x << "," << target_point.y 
                        << "), 检测框: [x=" << best_bbox.x << ", y=" << best_bbox.y 
                        << ", 宽=" << best_bbox.width << ", 高=" << best_bbox.height << "]"
                        << ", 是否在框内: " << (now_in_box ? "是" : "否") << std::endl;

                if (now_in_box) {
                    if (!center_in_box) {
                        center_in_box = true;
                        center_in_box_start_time = current_time;
                        victory_signal_sent = false;
                        outOfBoxCounter = 0;
                        std::cout << "基准点进入检测框，开始计时..." << std::endl;
                    } else {
                        //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        //    current_time - center_in_box_start_time
                        //);
                        auto duration = current_time - center_in_box_start_time;
                        //std::cout << duration.count() << std::endl;
                        //std::cout << victory_signal_sent << std::endl;
                        if ( ((unsigned long long) duration.count()) >= VICTORY_DURATION //&& !victory_signal_sent
                        ) {
                            std::cout << "胜利条件满足！基准点在检测框中超过" << VICTORY_DURATION << "毫秒" << std::endl;
                            
                            if (serial_available) {
                                std::string victory_signal = "5000,5000\n";
                                if (serial.send(victory_signal)) {
                                    std::cout << "胜利信号已发送: " << victory_signal;
                                    victory_signal_sent = true;
                                    center_in_box_start_time = current_time; // 重置计时
                                }
                            }
                            
                        }
                    }
                } else {
                    if (center_in_box) {
                    
                        outOfBoxCounter = outOfBoxCounter + 1;
                        if (outOfBoxCounter >= OUTBOUND_THRESHOLD)
                        {
                    
                        std::cout << "基准点离开检测框，重置计时" << std::endl;
                        center_in_box = false;
                        victory_signal_sent = false;
                        center_in_box_start_time = current_time;
                    
                        outOfBoxCounter = 0;
                        }
                    
                    }
                }
            }

            // 绘制检测结果（可选显示）
            if (show_display)
            {
                const auto &class_names = detector.getClassNames();
                for (const auto &result : results)
                {
                    cv::Scalar color = (result.score > 0.5f) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0);
                    // 为最大框添加特殊标记（如红色边框）
                    if (result.bbox == best_bbox) {
                        color = cv::Scalar(0, 0, 255); // 最大框用红色
                    }
                    cv::rectangle(frame, result.bbox, color, 2);
                    cv::circle(frame, result.center, 5, cv::Scalar(0, 0, 255), -1);
                    std::string text = class_names[result.class_id] + ": " +
                                    std::to_string(static_cast<int>(result.score * 100)) + "%";
                    cv::putText(frame, text, cv::Point(result.bbox.x, result.bbox.y - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }

                // 绘制画面中心点
                cv::Point frame_center(frame.cols/2, frame.rows/2);
                cv::line(frame, 
                        cv::Point(frame_center.x - 20, frame_center.y), 
                        cv::Point(frame_center.x + 20, frame_center.y), 
                        cv::Scalar(0, 0, 255), 2);
                cv::line(frame, 
                        cv::Point(frame_center.x, frame_center.y - 20), 
                        cv::Point(frame_center.x, frame_center.y + 20), 
                        cv::Scalar(0, 0, 255), 2);

                // 显示状态信息
                std::string state_text;
                switch (current_state) {
                    case ProgramState::WAITING_FOR_START: state_text = "等待开始信号"; break;
                    case ProgramState::DETECTING: 
                        if (center_in_box) {
                            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                                current_time - center_in_box_start_time
                            );
                            state_text = "检测中 - 已在框内: " + std::to_string(duration.count()) + "ms";
                            if (victory_signal_sent) {
                                state_text += " (已发送信号，重新计时中)";
                            }
                        } else {
                            state_text = "检测中 - 等待进入框内";
                        }
                        break;
                }

                cv::putText(frame, state_text, cv::Point(10, 30), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                cv::imshow("Detection", frame);
                if (cv::waitKey(1) == 27)
                {
                    break;
                }
            }
        }

        // 按间隔发送坐标
        if (serial_available)
        {
            auto current_time = std::chrono::system_clock::now();
            std::chrono::duration<double> time_since_last_send = current_time - last_send_time;
            
            if (time_since_last_send.count() >= SEND_INTERVAL)
            {
                std::string coord;
                if (has_high_confidence_detection && !centers.empty())
                {
                    std::stringstream ss;
                    ss << centers[0].x << "," << centers[0].y << "\n";
                    coord = ss.str();
                }
                else
                {
                    coord = "-1,-1\n";
                }

                if (serial.send(coord))
                {
                    if (is_detection_frame)
                    {
                        auto current_time_log = std::chrono::system_clock::now();
                        std::chrono::duration<double> fps_duration = current_time_log - prev_time;
                        double fps = 1.0 / fps_duration.count();
                        
                        std::cout << "FPS: " << std::fixed << std::setprecision(2) << fps
                                << ", Send to Arduino: " << coord.substr(0, coord.size() - 1) << std::endl;
                    }
                    last_send_time = current_time;
                }
                else
                {
                    std::cerr << "发送坐标失败: " << coord.substr(0, coord.size() - 1) << std::endl;
                }
            }
        }
    }

    // 释放资源
    serial.close();
    camera.release();
    if (show_display)
    {
        cv::destroyAllWindows();
    }

    return 0;
}
