#include "yolov5_detector.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>

#define RDK_CHECK_SUCCESS(value, errmsg)                        \
    do {                                                       \
        auto ret_code = value;                                 \
        if (ret_code != 0) {                                   \
            std::cout << errmsg << ", error code:" << ret_code; \
            return false;                                      \
        }                                                      \
    } while (0);

YOLOv5Detector::YOLOv5Detector(const std::string& model_path, 
                               float score_threshold, 
                               float nms_threshold,
                               int classes_num)
    : model_path_(model_path), 
      score_threshold_(score_threshold),
      nms_threshold_(nms_threshold),
      classes_num_(classes_num) {
    
    class_names_ = {"tag"};
    output_tensors_ = nullptr;
}

YOLOv5Detector::~YOLOv5Detector() {
    if (output_tensors_) {
        for (int i = 0; i < output_count_; i++) {
            hbSysFreeMem(&(output_tensors_[i].sysMem[0]));
        }
        delete[] output_tensors_;
    }
    
    if (packed_dnn_handle_) {
        hbDNNRelease(packed_dnn_handle_);
    }
}

bool YOLOv5Detector::initialize() {
    return loadModel();
}

bool YOLOv5Detector::loadModel() {
    // 加载模型
    const char* model_file_name = model_path_.c_str();
    RDK_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_file_name, 1),
        "hbDNNInitializeFromFiles failed");

    // 获取模型句柄
    const char** model_name_list;
    int model_count = 0;
    RDK_CHECK_SUCCESS(
        hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_),
        "hbDNNGetModelNameList failed");

    const char* model_name = model_name_list[0];
    RDK_CHECK_SUCCESS(
        hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name),
        "hbDNNGetModelHandle failed");

    // 获取输入属性
    RDK_CHECK_SUCCESS(
        hbDNNGetInputTensorProperties(&input_properties_, dnn_handle_, 0),
        "hbDNNGetInputTensorProperties failed");

    input_height_ = input_properties_.validShape.dimensionSize[2];
    input_width_ = input_properties_.validShape.dimensionSize[3];

    // 获取输出属性
    RDK_CHECK_SUCCESS(
        hbDNNGetOutputCount(&output_count_, dnn_handle_),
        "hbDNNGetOutputCount failed");
    output_tensors_ = new hbDNNTensor[output_count_];
    for (int i = 0; i < output_count_; i++) {
        hbDNNTensorProperties output_properties;
        RDK_CHECK_SUCCESS(
            hbDNNGetOutputTensorProperties(&output_properties, dnn_handle_, i),
            "hbDNNGetOutputTensorProperties failed");
        
        output_tensors_[i].properties = output_properties;
        int out_aligned_size = output_properties.alignedByteSize;
        hbSysAllocCachedMem(&(output_tensors_[i].sysMem[0]), out_aligned_size);
    }

    // 设置输出顺序
    int32_t H_8 = input_height_ / 8;
    int32_t H_16 = input_height_ / 16;
    int32_t H_32 = input_height_ / 32;
    int32_t W_8 = input_width_ / 8;
    int32_t W_16 = input_width_ / 16;
    int32_t W_32 = input_width_ / 32;
    
    int32_t order_we_want[6][3] = {
        {H_8, W_8, 3 * (5 + classes_num_)},
        {H_16, W_16, 3 * (5 + classes_num_)},
        {H_32, W_32, 3 * (5 + classes_num_)},
    };
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            hbDNNTensorProperties output_properties;
            hbDNNGetOutputTensorProperties(&output_properties, dnn_handle_, j);
            int32_t h = output_properties.validShape.dimensionSize[1];
            int32_t w = output_properties.validShape.dimensionSize[2];
            int32_t c = output_properties.validShape.dimensionSize[3];
            if (h == order_we_want[i][0] && w == order_we_want[i][1] && c == order_we_want[i][2]) {
                order_[i] = j;
                break;
            }
        }
    }

    
    return true;
}

void YOLOv5Detector::prepareInput(const cv::Mat& img, bool letterbox) {
    auto begin_time = std::chrono::system_clock::now();

    if (letterbox) {
        // LetterBox处理
        x_scale_ = std::min(1.0f * input_width_ / img.cols, 1.0f * input_height_ / img.rows);
        y_scale_ = x_scale_;
        
        int new_w = img.cols * x_scale_;
        x_shift_ = (input_width_ - new_w) / 2;
        int new_h = img.rows * y_scale_;
        y_shift_ = (input_height_ - new_h) / 2;
        
        cv::Size targetSize(new_w, new_h);
        cv::resize(img, resize_img_, targetSize);
        
        int x_other = input_width_ - new_w - x_shift_;
        int y_other = input_height_ - new_h - y_shift_;
        cv::copyMakeBorder(resize_img_, resize_img_, 
                          y_shift_, y_other, x_shift_, x_other, 
                          cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));
    } else {
        // Resize处理
        cv::Size targetSize(input_width_, input_height_);
        cv::resize(img, resize_img_, targetSize);
        
        x_scale_ = 1.0f * input_width_ / img.cols;
        y_scale_ = 1.0f * input_height_ / img.rows;
        x_shift_ = 0;
        y_shift_ = 0;
    }

    // 转换为NV12格式
    cv::Mat yuv_mat;
    cv::cvtColor(resize_img_, yuv_mat, cv::COLOR_BGR2YUV_I420);
    uint8_t* yuv = yuv_mat.ptr<uint8_t>();
    
    cv::Mat img_nv12(input_height_ * 3 / 2, input_width_, CV_8UC1);
    uint8_t* ynv12 = img_nv12.ptr<uint8_t>();
    
    int uv_height = input_height_ / 2;
    int uv_width = input_width_ / 2;
    int y_size = input_height_ * input_width_;
    
    memcpy(ynv12, yuv, y_size);
    uint8_t* nv12 = ynv12 + y_size;
    uint8_t* u_data = yuv + y_size;
    uint8_t* v_data = u_data + uv_height * uv_width;
    
    for (int i = 0; i < uv_width * uv_height; i++) {
        *nv12++ = *u_data++;
        *nv12++ = *v_data++;
    }

    // 准备输入张量
    hbSysAllocCachedMem(&input_tensor_.sysMem[0], int(3 * input_height_ * input_width_ / 2));
    memcpy(input_tensor_.sysMem[0].virAddr, ynv12, int(3 * input_height_ * input_width_ / 2));
    hbSysFlushMem(&input_tensor_.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    input_tensor_.properties = input_properties_;

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end_time - begin_time;
    std::cout << "Prepare input time: " << elapsed.count() << "s" << std::endl;
}

std::vector<YOLOv5Detector::DetectionResult> YOLOv5Detector::detect(const cv::Mat& img, bool letterbox) {
    prepareInput(img, letterbox);

    // 执行推理
    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    hbDNNInfer(&task_handle, &output_tensors_, &input_tensor_, dnn_handle_, &infer_ctrl_param);
    hbDNNWaitTaskDone(task_handle, 0);
    hbDNNReleaseTask(task_handle);

    // 后处理
    auto results = postProcess();

    // 释放输入内存
    hbSysFreeMem(&(input_tensor_.sysMem[0]));

    return results;
}

std::vector<YOLOv5Detector::DetectionResult> YOLOv5Detector::postProcess() {
    auto begin_time = std::chrono::system_clock::now();
    float CONF_THRES_RAW = -log(1 / score_threshold_ - 1);
    
    std::vector<std::vector<cv::Rect2d>> bboxes(classes_num_);
    std::vector<std::vector<float>> scores(classes_num_);
    std::vector<std::vector<DetectionResult>> all_results(classes_num_);

    // 处理三个输出层
    for (int output_idx = 0; output_idx < 3; output_idx++) {
        int order = order_[output_idx];
        hbSysFlushMem(&(output_tensors_[order].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
        
        int stride = (output_idx == 0) ? 8 : (output_idx == 1) ? 16 : 32;
        int height = input_height_ / stride;
        int width = input_width_ / stride;
        
        auto* raw_data = reinterpret_cast<float*>(output_tensors_[order].sysMem[0].virAddr);
        
        for (int h = 0; h < height; h++) {
            for (int w = 0; w < width; w++) {
                for (int a = 0; a < 3; a++) {
                    float* cur_raw = raw_data;
                    raw_data += (5 + classes_num_);
                    
                    if (cur_raw[4] < CONF_THRES_RAW) continue;
                    
                    // 找到最大类别分数
                    int cls_id = 5;
                    for (int i = 6; i < 5 + classes_num_; i++) {
                        if (cur_raw[i] > cur_raw[cls_id]) {
                            cls_id = i;
                        }
                    }
                    float score = 1.0f / (1.0f + std::exp(-cur_raw[4])) / 
                                 (1.0f + std::exp(-cur_raw[cls_id]));
                    
                    if (score < score_threshold_) continue;
                    cls_id -= 5;
                    
                    // 解码边界框
                    float center_x = ((1.0f / (1.0f + std::exp(-cur_raw[0]))) * 2 - 0.5f + w) * stride;
                    float center_y = ((1.0f / (1.0f + std::exp(-cur_raw[1]))) * 2 - 0.5f + h) * stride;
                    float bbox_w = std::pow((1.0f / (1.0f + std::exp(-cur_raw[2]))) * 2, 2) * 
                                  ((output_idx == 0) ? 10.0f : (output_idx == 1) ? 30.0f : 116.0f);
                    float bbox_h = std::pow((1.0f / (1.0f + std::exp(-cur_raw[3]))) * 2, 2) * 
                                  ((output_idx == 0) ? 13.0f : (output_idx == 1) ? 61.0f : 90.0f);
                    
                    float bbox_x = center_x - bbox_w / 2.0f;
                    float bbox_y = center_y - bbox_h / 2.0f;
                    
                    // 转换到原始图像坐标
                    float x1 = (bbox_x - x_shift_) / x_scale_;
                    float y1 = (bbox_y - y_shift_) / y_scale_;
                    float x2 = x1 + bbox_w / x_scale_;
                    float y2 = y1 + bbox_h / y_scale_;
                    
                    // 创建检测结果
                    DetectionResult result;
                    result.bbox = cv::Rect2d(x1, y1, x2 - x1, y2 - y1);
                    result.score = score;
                    result.class_id = cls_id;
                    result.center = cv::Point2f((x1 + x2) / 2, (y1 + y2) / 2);
                    
                    all_results[cls_id].push_back(result);
                }
            }
        }
    }

    // 对每个类别进行NMS
    std::vector<DetectionResult> final_results;
    for (int cls_id = 0; cls_id < classes_num_; cls_id++) {
        std::vector<cv::Rect2d> bboxes;
        std::vector<float> scores;
        std::vector<int> indices;
        
        for (const auto& result : all_results[cls_id]) {
            bboxes.push_back(result.bbox);
            scores.push_back(result.score);
        }
        
        if (!bboxes.empty()) {
            cv::dnn::NMSBoxes(bboxes, scores, score_threshold_, nms_threshold_, indices, 1.f, 300);
            
            for (int idx : indices) {
                final_results.push_back(all_results[cls_id][idx]);
            }
        }
    }
    
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end_time - begin_time;
    std::cout << "Post process time: " << elapsed.count() << "s" << std::endl;

    return final_results;
}

void YOLOv5Detector::printDetectionResults(const std::vector<DetectionResult>& results) {
    for (const auto& result : results) {
        std::cout << "Class: " << class_names_[result.class_id] 
                  << ", Score: " << result.score 
                  << ", Center: (" << result.center.x << ", " << result.center.y << ")"
                  << ", BBox: [" << result.bbox.x << ", " << result.bbox.y 
                  << ", " << result.bbox.width << ", " << result.bbox.height << "]"
                  << std::endl;
    }
}
