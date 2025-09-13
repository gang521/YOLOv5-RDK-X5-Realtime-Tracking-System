# 基于 YOLOv5 的 RDK X5 实时目标追踪系统

## 项目简介

该项目实现了一个基于 **YOLOv5** 的目标检测与追踪系统，部署于 **地平线 RDK X5 嵌入式平台**，并通过串口与 Arduino 进行通信，结合激光打击、运动控制等功能，实现实时目标检测、坐标传输、胜利判定及控制指令输出。

系统架构包含：

* **相机模块**：基于海康相机 SDK，直接采集 Bayer 格式视频流，解码后输入检测器。
* **推理模块**：YOLOv5 PTQ 量化模型加载与多线程推理，检测 FPS 达 400+。
* **通信模块**：串口通信，实时向 Arduino 发送检测目标的中心坐标或胜利信号。
* **控制逻辑**：实现开始信号等待、目标框判定、胜利条件计时、视觉-控制延时优化。

## 主要功能

* 连接海康相机并持续抓取帧，自动转换 Bayer → BGR 图像。
* 基于 YOLOv5 量化模型的高性能目标检测，自动筛选最大检测框。
* 串口通信模块支持发送坐标、胜利信号。
* 可选显示检测结果、帧率及状态信息，便于调试。
* 当目标中心持续在检测框内超过设定时间，发送胜利信号给 Arduino 控制激光或执行其他动作。

## 代码结构

```
├── README.md
├── CMakeLists.txt
├── build
├── models
│  ├─ yolov5n_new_nv12.bin
├── include
│  ├─ camera_yolo.hpp
│  ├─ serial_port.hpp
│  ├─ yolov5_detector.hpp 
├── src
├── main.cpp              # 主程序入口，包含状态机、检测循环、通信逻辑
│  ├─ camera_yolo.cpp       # 海康相机驱动封装，Bayer 流解码
│  ├─ yolov5_detector.cpp   # YOLOv5 模型加载、推理与后处理
│  ├─ serial_port.cpp       # 串口通信类，封装 open/close/send 接口
│  ├─ test.cpp              # 串口测试程序，简单验证串口打开/关闭
```

## 运行方式

### 1. 编译项目

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 2. 部署模型

将量化好的 `yolov5n_nv12.bin` 模型放入 `models/` 文件夹，并在 `main.cpp` 中修改 `MODEL_PATH` 为对应路径。

### 3. 运行程序

```bash
./camera_yolov5_detection --display   # 开启显示窗口
```

程序会：

* 等待 Arduino 通过串口发送字符 `'1'` 启动检测
* 实时读取相机图像、执行目标检测
* 将检测中心坐标通过串口发送给 Arduino
* 满足胜利条件时，发送胜利信号

## 硬件环境

* RDK X5 开发板（支持多线程 DNN 推理）
* 海康 CCD 相机（支持 BayerGB8 输出）
* Arduino Mega2560（负责接收坐标、控制激光）

## 性能指标

* 7 线程 YOLOv5n 推理：≈ 460 FPS，单帧延时 ≈ 0.2 ms
* 视觉-控制协同延时 < 0.1 s
* 检测框角度误差 ≤ 0.5°，差速轮速度误差 ≤ 5%

## 调试建议

* 若串口无法打开，检查 `/dev/ttyACM0` 权限或替换为实际端口。
* 可使用 `test.cpp` 单独测试串口功能。
* 使用 `--display` 参数观察实时检测效果与状态变化。

## TODO / 可扩展

* 支持多目标追踪与优先级选择
* 增加 ROS2 节点接口，便于集成到更大系统
* 优化串口协议，支持双向通信与状态回传

## 断网运行
sudo ./camera_yolov5_detection > /dev/null 2>&1 &

