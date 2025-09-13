#ifndef CAMERA_YOLO_HPP
#define CAMERA_YOLO_HPP

#include <opencv2/opencv.hpp>
#include "/opt/MVS/include/MvCameraControl.h"

class HikCamera {
public:
    HikCamera();
    ~HikCamera();

    bool connect(int device_index = 0);
    bool read(cv::Mat& image);
    void release();

private:
    void* camHandle;
    bool connected;
    unsigned int payloadSize;
};

#endif // CAMERA_YOLO_HPP