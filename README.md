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
1. 导出onnx模型
方法一：在已有环境内手动导出
# 进入文件夹/
cd /media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/ultralytics

# 进入对应环境
# 这是没有下载ultralytics环境的包
conda activate yolo
# 这是下载了ultralytics的包（训练的时候切换这个环境，导出onnx的时候用yolo环境）
# conda activate YS
在ultralytics/nn/modules/block.py内找到class Attention的forward函数和forward_函数：
[图片]
标了 # RDK yolo 解释的（图中左侧有绿色）是用于导出onnx模型的，将这个函数命名为forward，将原本的forward函数改名为forward_。
ultralytics/nn/modules/head.py找到class Detect 和 class Segment中找到同样的forward_和forward（如下图），解释中标注了 # RDK 的是用于导出onnx的，改名为forward，将原本的forward命名为forward_。
[图片]
在终端（yolo环境）：
# 以下代码在yolo环境运行
python3
from ultralytics import YOLO
model = YOLO("要量化的pt模型.pt")
model.export(imgsz=640, format='onnx', simplify=False, opset=11)
# opset=11是必需的，imgsz可以自己改为需要的
  输出应该如下：
(yolo)pc00036@pc00036-System-Product-Name:/media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/ultralytics$ python3
Python 3.11.13 (main, Jun  5 2025, 13:12:00) [GCC 11.2.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> from ultralytics import YOLO
>>> model = YOLO("yolo11n.pt")
>>> model.export(imgsz=640, format='onnx', simplify=False, opset=11)
Ultralytics 8.3.176 🚀 Python-3.11.13 torch-2.9.0.dev20250803+cu129 CPU (13th Gen Intel Core(TM) i9-13900KF)
YOLO11n summary (fused): 100 layers, 2,616,248 parameters, 0 gradients

PyTorch: starting from 'yolo11n.pt' with input shape (1, 3, 640, 640) BCHW and output shape(s) (1, 84, 8400) (5.4 MB)

ONNX: starting export with onnx 1.17.0 opset 11...
ONNX: export success ✅ 0.3s, saved as 'yolo11n.onnx' (10.2 MB)

Export complete (0.5s)
Results saved to /media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/ultralytics
Predict:         yolo predict task=detect model=yolo11n.onnx imgsz=640  
Validate:        yolo val task=detect model=yolo11n.onnx imgsz=640 data=/usr/src/ultralytics/ultralytics/cfg/datasets/coco.yaml  
Visualize:       https://netron.app
'yolo11n.onnx'
以上修改，如果要进行新模型的训练，需要将forward函数改为原本自带的那个，将新增的# rdk再次改回forward_，防止训练出现异常。
方法二：官方脚本
# 进入下载了脚本的文件夹
cd /media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb
#进入安装了ultralytics的环境
conda activate YS
# 运行导出脚本
python3 export_monkey_patch.py --pt yolo11n.pt
输出类似：
(YS)pc00036@pc00036-System-Product-Name:/media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb$ conda list | grep ultralytics
ultralytics                 8.3.176          pypi_0              pypi
ultralytics-thop            2.0.15           pypi_0              pypi
(YS)pc00036@pc00036-System-Product-Name:/media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb$ python3 export_monkey_patch.py --pt yolo11n.pt
[Cauchy] Replaced Attention_forward in attn
[Cauchy] Replaced Detect_forward in 23
Ultralytics 8.3.176 🚀 Python-3.13.5 torch-2.8.0+cu128 CPU (13th Gen Intel Core(TM) i9-13900KF)
YOLO11n summary (fused): 100 layers, 2,616,248 parameters, 0 gradients, 6.5 GFLOPs

PyTorch: starting from 'yolo11n.pt' with input shape (1, 3, 640, 640) BCHW and output shape(s) ((1, 80, 80, 80), (1, 80, 80, 64), (1, 40, 40, 80), (1, 40, 40, 64), (1, 20, 20, 80), (1, 20, 20, 64)) (5.4 MB)

ONNX: starting export with onnx 1.17.0 opset 11...
ONNX: export success ✅ 0.3s, saved as 'yolo11n.onnx' (10.0 MB)

Export complete (0.6s)
Results saved to /media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb
Predict:         yolo predict task=detect model=yolo11n.onnx imgsz=640  
Validate:        yolo val task=detect model=yolo11n.onnx imgsz=640 data=/usr/src/ultralytics/ultralytics/cfg/datasets/coco.yaml  
Visualize:       https://netron.ap
2. （手动量化）量化的docker环境
进入docker
# 加载docker
docker load -i /media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb/docker_openexplorer_ubuntu_20_x5_cpu_v1.2.8.tar.gz
#应该输出：Loaded image: openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8-py310
# 进入docker
# 把dota8文件夹挂载为data，把video_detect挂载为code
docker run -it --rm -v /media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb/dota8:/data -v /media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb/video_detect:/code openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8-py310
准备校准数据
# 进入data
root@69e708fe9168:/open_explorer# cd ../data
从模型训练集和验证集选择约100张图片传入yoloobb/dota8文件夹
打开generate_calibration_data.py，修改40行的位置的以下内容：
parser = argparse.ArgumentParser()
# 你传入的原始校准数据集的文件夹位置（相对于data目录）
parser.add_argument('--src', type=str, default='img', help="Source images path") 
# 生成的校准数据集的名字
parser.add_argument('--dist', type=str, default='calibration_data_rgb_f32_640', help="Destination images path") 
# 图像比例（默认640x640，与之前导出onnx模型时的imgsz保持一致）
parser.add_argument('--width', type=int, default=640, help="W in ONNX NCHW.") 
parser.add_argument('--height', type=int, default=640, help="H in ONNX NCHW.")
# 最终生成的是NCHW维度的、float32格式的二进制张量
如果所用的模型需要的预处理与generate_calibration_data.py设置的预处理方式不一致，手动修改并生成合适的校准数据。
如果对量化的精度要求不高，只是先跑通量化，可以先跳过这一步校准数据，在后续编写量化的yaml文件时把preprocess参数设置为True即可（当然这会导致精度受损）
hb_mapper_checker检查onnx模型
在docker环境内：进入挂载的code文件夹运行
hb_mapper checker --model-type onnx --march bayes-e --model yolo11n-seg.onnx
这里的hb_mapper checker命令：
  hb_mapper checker --model-type ${model_type} \ # 设置为onnx即可
                    --march ${march} \ # rdk X5设置为 bayes-e
                    --model ${caffe_model/onnx_model} \ # 模型名称和位置
                    --input-shape ${input_node} ${input_shape} \ # 可以不设置，设置时取值为模型输入头名称和对应的shape，如： --input-shape data1 1x224x224x3
正常不存在error时会输出模型的检查结果，需要关注：
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
2. node信息
这里理想情况下，ON列会全部显示BPU，即所有节点都可以在BPU运行
Node                                         ON   Subgraph  Type                       Cosine Similarity  Threshold  DataType  
-------------------------------------------------------------------------------------------------------------------------------
/model.0/conv/Conv                           BPU  id(0)     HzSQuantizedConv           --                 1.0        int8      
/model.0/act/Mul                             BPU  id(0)     HzLut                      --                 1.0        int8      
/model.1/conv/Conv                           BPU  id(0)     HzSQuantizedConv           --                 1.0        int8      
/model.1/act/Mul                             BPU  id(0)     HzLut                      --                 1.0        int8      
/model.2/cv1/conv/Conv                       BPU  id(0)     HzSQuantizedConv           --                 1.0        int8      
/model.2/cv1/act/Mul                         BPU  id(0)     HzLut                      --                 1.0        int8      
/model.2/Split                               BPU  id(0)     Split                      --                 1.0        int8      
如果有节点出现了CPU，则可能导致后续运行速度下降，可以检查算子支持列表看该节点是否支持在BPU运行：https://developer.d-robotics.cc/rdk_doc/Advanced_development/toolchain_development/intermediate/supported_op_list，并根据checker命令时给出的导致出现CPU算子的原因进行修改
另外，如果第一步导出onnx的时候没有成功修改forward函数为对应内容，这里会出现softmax算子在CPU上运行，可以不用管，在下一步的yaml文件里面手动指定即可
配置量化转换的yaml文件
进入docker的code文件夹，其中的config_yolo11_seg_bayese_640x640_nv12.yaml文件是yolo11模型默认的yaml配置。
  需要修改的参数以下标出：
model_parameters:
  # 这是需要量化的onnx模型的位置
  onnx_model: 'yolo11n-seg.onnx'
  march: "bayes-e" # 适配rdk x5
  layer_out_dump: False
  # 转换结果输出的文件夹位置（没有会新建，建议每一个量化都改一个文件夹防止弄混）
  working_dir: 'yolo11n_seg_bayese_640x640_nv12'
  # 导出产物的前缀
  output_model_file_prefix: 'yolo11n_seg_bayese_640x640_nv12'
  # YOLO11 n, s, m
  # 如果前面没有成功修改forward出现softmax的CPU算子，这里可以手动指定在BPU上运行
  node_info: {"/model.10/m/m.0/attn/Softmax": {'ON': 'BPU','InputType': 'int16','OutputType': 'int16'}}
  # YOLO11 l, x
  # node_info: {"/model.10/m/m.0/attn/Softmax": {'ON': 'BPU','InputType': 'int16','OutputType': 'int16'},
  #             "/model.10/m/m.1/attn/Softmax": {'ON': 'BPU','InputType': 'int16','OutputType': 'int16'}}
input_parameters:
  # input_batch: 8
  # 输入节点名称，可以不用管
  input_name: ""
  # 转换后模型需要适配的输入格式（rgb,bgr,yuv444,gray,featuremap)
  input_type_rt: 'nv12'
  # 指定模型的输入数据类型（rgb,bgr,yuv444,gray,featuremap)
  input_type_train: 'rgb'
  # 输入数据排布（NCHW, NHWC）需保持与原始模型相同
  input_layout_train: 'NCHW'
  # 添加的数据预处理
  norm_type: 'data_scale'
  scale_value: 0.003921568627451
calibration_parameters:
  # 校准数据文件夹的位置（改为自己生成的那个）
  cal_data_dir: '../data/calibration_data_rgb_f32_640'
  # 适配好就行
  cal_data_type: 'float32'
  # 是否开启自动预处理（如果前面跳过了生成校准数据，这里必须设置为True
  # 如果生成了校准数据，不用设置该参数或者设置为flase
  # preprocess_on: False
compiler_parameters:
  # 编译策略（latency--优化推理时间，bandwidth--优化访问带宽）
  compile_mode: 'latency'
  # 是否打开编译的debug信息
  debug: False
  # 模型运行核心数（1，2）如果输入尺寸较大，可以用2，默认为1即可
  # core_num: 1
  # 编译优化等级（01,02,03)03编译后模型速度最快但是编译耗时最长
  optimize_level: 'O3'
hb_mapper_makertbin进行转换
在docker的code文件夹下，运行：
hb_mapper makertbin --config ${config_file}  \ # 配置的yaml文件
                      --model-type  ${model_type} # 用onnx即可
官方还有一个fast-perf模式，会导出删除了多余量化、反量化节点的模型，但是精度可能受损
hb_mapper makertbin --fast-perf --model ${caffe_model/onnx_model} --model-type ${model_type} \
                      --proto ${caffe_proto} \
                      --march ${march}
转换过程中会显示各个node的余弦相似度，越大越好，如果输出节点的cosine similarity 大于 0.99则可以认为量化正常。如果低于0.8则量化精度受损较为严重
转换产出的文件夹内部有：
***_original_float_model.onnx
***_optimized_float_model.onnx
***_calibrated_model.onnx
# 上面三个没什么用
***_quantized_model.onnx # 可以用来检查精度验证
***.bin # 这是最终要的量化模型！
模型性能检查和优化
1. 在x86主机上docker内部：
  进入转换产出的文件夹，运行：
hb_perf xxx.bin # 产出的bin模型
会在hb_perf_result文件夹内生成检查结果，其中的xxx.html给出分析情况，xxx.png给出了bin模型的结构。理想情况是输入结构经过BPU后直接到输出节点，但是会出现BPU节点出来后经过了Dequantize和Quantize节点再输出，这里可以进行优化。
  以yolo11n-seg模型为例，这是第一步生成的bin模型：
[图片]
  所有输出节点都经过了反量化节点，我们查找可以移除的节点：
# 查找可能可以移除的反量化和量化节点
hb_mapper_modifier xxx.bin
  例如：
root@69e708fe9168:/code/yolo11n_rgb# hb_model_modifier yolo11n_rgb.bin
2025-08-27 15:31:36,496 INFO log will be stored in /code/yolo11n_rgb/hb_model_modifier.log
2025-08-27 15:31:36,502 INFO Nodes that can be deleted: ['/model.23/cv3.0/cv3.0.2/Conv_output_0_HzDequantize', '/model.23/cv2.0/cv2.0.2/Conv_output_0_HzDequantize', '/model.23/cv4.0/cv4.0.2/Conv_output_0_HzDequantize', '/model.23/cv3.1/cv3.1.2/Conv_output_0_HzDequantize', '/model.23/cv2.1/cv2.1.2/Conv_output_0_HzDequantize', '/model.23/cv4.1/cv4.1.2/Conv_output_0_HzDequantize', '/model.23/cv3.2/cv3.2.2/Conv_output_0_HzDequantize', '/model.23/cv2.2/cv2.2.2/Conv_output_0_HzDequantize', '/model.23/cv4.2/cv4.2.2/Conv_output_0_HzDequantize', '585_HzDequantize']
输出了9个节点但是不是所有的都可以移除，需要在生成的log文件中找到大小为[1, 64, 80, 80], [1, 64, 40, 40], [1, 64, 20, 20]的三个输出头和[1, 80, 80, 32], [1, 40, 40, 32], [1, 20, 20, 32]三个输出头的名称, 还有[1, 160, 160, 32]的输出头名称，再进行移除：
hb_model_modifier yolo11n_rgb.bin \
-r /model.23/cv2.0/cv2.0.2/Conv_output_0_HzDequantize \
-r /model.23/cv2.1/cv2.1.2/Conv_output_0_HzDequantize \
-r /model.23/cv2.2/cv2.2.2/Conv_output_0_HzDequantize \
-r /model.23/cv4.0/cv4.0.2/Conv_output_0_HzDequantize \
-r /model.23/cv4.1/cv4.1.2/Conv_output_0_HzDequantize \
-r /model.23/cv4.2/cv4.2.2/Conv_output_0_HzDequantize \
-r 585_HzDequantize
得到的yolo11n_rgb_modified.bin模型再次用hb_perf进行可视化检查：
[图片]
剩下的三个节点是进行分类的检测头（这里默认是coco数据集，80个类别）。
2. 在rdk x5板端：
先参考https://developer.d-robotics.cc/rdk_doc/Algorithm_Application/model_zoo/model_zoo_intro进行安装bpu_nfer_lib
pip install bpu_infer_lib_x5 -i http://sdk.d-robotics.cc:8080/simple/ --trusted-host sdk.d-robotics.cc
采用/media/pc00036/2115403e-0ae5-49e3-9e54-d4020d4442ef/yoloobb/Ai_Toolchain_Package-release-v1.23.8-OE-v1.2.6/package/board这个脚本进行安装：
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
3. （脚本量化）官方更新的脚本工具
在docker环境的code文件夹内有mapper.py，修改对应的参数为需要值并运行即可。
基本参数改为需要的
parser = argparse.ArgumentParser()
# 校准数据位置
parser.add_argument('--cal-images', type=str, default='./cal_images', help='*.jpg, *.png calibration images path, 20 ~ 50 pictures is OK.') 
# 模型位置
parser.add_argument('--onnx', type=str, default='./yolo11n.onnx', help='origin float onnx model path.')
# 生成文件位置
parser.add_argument('--output-dir', type=str, default='.', help='output directory for converted model.')
# default below
parser.add_argument('--quantized', type=str, default="int8", help='int8 first / int16 first')
parser.add_argument('--jobs', type=int, default=16, help='model combine jobs.')
parser.add_argument('--optimize-level', type=str, default='O3', help='O0, O1, O2, O3')
# 是否从校准图中随机采样
parser.add_argument('--cal-sample', type=bool, default=True, help='sample calibration data or not.') 
# 采样后的校准图数量
parser.add_argument('--cal-sample-num', type=int, default=20, help='num of sample calibration data.') 
parser.add_argument('--save-cache', type=bool, default=False, help='remove bpu output files or not.') 
# private settings
# 临时校准数据文件夹名（存放预处理后的.rgbchw 文件）
parser.add_argument('--cal', type=str, default='.calibration_data_temporary_folder', help='calibration_data_temporary_folder')
# 临时工作目录（存放配置文件、中间模型、BPU 编译输出）
parser.add_argument('--ws', type=str, default='.temporary_workspace', help='temporary workspace')
opt = parser.parse_args()
yaml文件内容参考量化工具进行配置，并且默认生成的名为config.yaml，可以修改
 # 生成的yaml文件内容
    yaml_content = f'''model_parameters:
  onnx_model: '{opt.onnx}'
  march: "bayes-e"
  layer_out_dump: False
  working_dir: '{bpu_output_dir}'
  output_model_file_prefix: '{output_model_prefix}'
input_parameters:
  input_name: ""
  input_type_rt: 'nv12'
  input_type_train: 'rgb'
  input_layout_train: 'NCHW'
  norm_type: 'data_scale'
  scale_value: 0.003921568627451
calibration_parameters:
  cal_data_dir: '{cal_data_dir}'
  cal_data_type: 'float32'
  calibration_type: 'default'
  optimization: set_Softmax_input_int8,set_Softmax_output_int8{int16_config_str}
compiler_parameters:
  jobs: {opt.jobs}
  compile_mode: 'latency'
  debug: true
  optimize_level: '{opt.optimize_level}'
'''
脚本默认为640x640尺寸，如果onnx模型输入不是640，在代码中修改width、height参数改为与模型一致的值
脚本固定了预处理流程为：BGR→RGB→Resize→HWC 转 CHW→加 Batch 维度→转 FLOAT32。如果训练时进行了特殊的预处理，需要在270行左右进行修改（比如加上归一化等）
确认无误后运行：
python3 mapper.py --onnx [*.onnx] --cal-images [cal images path]
模型测试
在rdk x5板端进行
检查量化模型是否正常可以采用这里的脚本进行验证（这里只要.py)
https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/Vision/ultralytics_YOLO/py
更多的运行代码（包括c)看这里
https://github.com/D-Robotics/rdk_model_zoo


有问题看这里：
https://developer.d-robotics.cc/rdk_doc/Advanced_development/toolchain_development/intermediate/ptq_process

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

