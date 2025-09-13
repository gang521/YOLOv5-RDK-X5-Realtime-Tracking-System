#ifndef YOLOV5_DETECTOR_HPP
#define YOLOV5_DETECTOR_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"

class YOLOv5Detector {
public:
    struct DetectionResult {
        cv::Rect2d bbox;
        float score;
        int class_id;
        cv::Point2f center;  // 中心点坐标
    };

    YOLOv5Detector(const std::string& model_path, 
                   float score_threshold = 0.5f, 
                   float nms_threshold = 0.6f,
                   int classes_num = 1);
    ~YOLOv5Detector();

    bool initialize();
    std::vector<DetectionResult> detect(const cv::Mat& img, bool letterbox = true);
    void printDetectionResults(const std::vector<DetectionResult>& results);
    const std::vector<std::string>& getClassNames() const { return class_names_; }

private:
    bool loadModel();
    void prepareInput(const cv::Mat& img, bool letterbox);
    std::vector<DetectionResult> postProcess();

    // 模型相关参数
    std::string model_path_;
    float score_threshold_;
    float nms_threshold_;
    int classes_num_;
    std::vector<std::string> class_names_;

    // BPU相关
    hbPackedDNNHandle_t packed_dnn_handle_;
    hbDNNHandle_t dnn_handle_;
    hbDNNTensorProperties input_properties_;
    hbDNNTensor input_tensor_;
    hbDNNTensor* output_tensors_;
    int output_count_;
    int order_[3];

    // 图像处理相关
    cv::Mat resize_img_;
    float x_scale_;
    float y_scale_;
    int x_shift_;
    int y_shift_;
    int input_height_;
    int input_width_;
};

#endif // YOLOV5_DETECTOR_HPP