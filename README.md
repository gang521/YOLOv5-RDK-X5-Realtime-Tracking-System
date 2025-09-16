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

### 2. 部署量化的模型

模型量化参考了RDK官方的GitHub教程：
1. 输入输出情况
  
Graph input:
    images:               shape=[1, 3, 640, 640], dtype=FLOAT32
Graph output:
    output0:              shape=[1, 80, 80, 80], dtype=FLOAT32
    output1:              shape=[1, 80, 80, 64], dtype=FLOAT32
    514:                  shape=[1, 80, 80, 32], dtype=FLOAT32
    528:                  shape=[1, 40, 40, 80], dtype=FLOAT32
    536:                  shape=[1, 40, 40, 64], dtype=FLOAT32
    544:                  shape=[1, 40, 40, 32], dtype=FLOAT32
    558:                  shape=[1, 20, 20, 80], dtype=FLOAT32
    566:                  shape=[1, 20, 20, 64], dtype=FLOAT32
    574:                  shape=[1, 20, 20, 32], dtype=FLOAT32
    585:                  shape=[1, 160, 160, 32], dtype=FLOAT32
在rdk x5板端：
先参考https://developer.d-robotics.cc/rdk_doc/Algorithm_Application/model_zoo/model_zoo_intro进行安装bpu_nfer_lib
pip install bpu_infer_lib_x5 -i http://sdk.d-robotics.cc:8080/simple/ --trusted-host sdk.d-robotics.cc
bash install_ultra.sh ${board_ip}
之后可以用scp命令将转换的bin模型传入x5，进行验证：
hrt_model_exec perf --model_file xxx.bin \
                      --model_name="" \
                      --core_id=0 \
                      --frame_count=200 \ # 推理帧数
                      --perf_time=0 \
                      --thread_num=1 \ # 运行的线程数，x5可以改为1-8
                      --profile_path="." #生成的日志文件的位置
会生成详细的分析日志。
板端验证
https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/Vision/ultralytics_YOLO/py
更多的运行代码（包括c)看这里
https://github.com/D-Robotics/rdk_model_zoo
有问题看这里：
https://developer.d-robotics.cc/rdk_doc/Advanced_development/toolchain_development/intermediate/ptq_process

板端部署运行
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

* 7 线程 YOLOv5n 推理：≈ 460 FPS
* 1 线程 约 250 FPS
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
为符合比赛要求 可采用断网运行
sudo ./camera_yolov5_detection > /dev/null 2>&1 &

