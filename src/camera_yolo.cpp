#include "camera_yolo.hpp"
#include <iostream>
#include <cstring>

#define PixelType_Gvsp_BayerGB8 0x0108000A
#define PixelType_Gvsp_Mono8 0x01080001

HikCamera::HikCamera() : camHandle(nullptr), connected(false), payloadSize(0) {}

HikCamera::~HikCamera() {
    release();
}

bool HikCamera::connect(int device_index) {
    int ret = MV_CC_Initialize();
    if (ret != MV_OK) {
        std::cerr << "MV_CC_Initialize failed, ret = " << std::hex << ret << std::endl;
        return false;
    }

    MV_CC_DEVICE_INFO_LIST devList;
    memset(&devList, 0, sizeof(devList));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devList);
    if (ret != MV_OK || devList.nDeviceNum == 0) {
        std::cerr << "MV_CC_EnumDevices failed or no devices found, ret = " << std::hex << ret << std::endl;
        return false;
    }

    // 修改这里：将 device_index 转换为 unsigned int 进行比较
    if (static_cast<unsigned int>(device_index) >= devList.nDeviceNum) {
        std::cerr << "Device index out of range" << std::endl;
        return false;
    }

    MV_CC_DEVICE_INFO* deviceInfo = devList.pDeviceInfo[device_index];
    if (!deviceInfo) {
        std::cerr << "deviceInfo is null." << std::endl;
        return false;
    }

    ret = MV_CC_CreateHandle(&camHandle, deviceInfo);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_CreateHandle failed, ret = " << std::hex << ret << std::endl;
        return false;
    }

    ret = MV_CC_OpenDevice(camHandle);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_OpenDevice failed, ret = " << std::hex << ret << std::endl;
        return false;
    }

    ret = MV_CC_SetEnumValue(camHandle, "PixelFormat", PixelType_Gvsp_BayerGB8);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_SetEnumValue failed, ret = " << std::hex << ret << std::endl;
        return false;
    }

    MVCC_INTVALUE payload;
    memset(&payload, 0, sizeof(payload));
    ret = MV_CC_GetIntValue(camHandle, "PayloadSize", &payload);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_GetIntValue PayloadSize failed, ret = " << std::hex << ret << std::endl;
        return false;
    }
    payloadSize = payload.nCurValue;

    ret = MV_CC_SetEnumValue(camHandle, "TriggerMode", 0);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_SetEnumValue TriggerMode failed, ret = " << std::hex << ret << std::endl;
        return false;
    }

    ret = MV_CC_StartGrabbing(camHandle);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_StartGrabbing failed, ret = " << std::hex << ret << std::endl;
        return false;
    }

    connected = true;
    return true;
}

bool HikCamera::read(cv::Mat& image) {
    if (!connected || !camHandle) {
        return false;
    }

    MV_FRAME_OUT frame;
    memset(&frame, 0, sizeof(frame));
    int ret = MV_CC_GetImageBuffer(camHandle, &frame, 1000);
    if (ret != MV_OK) {
        return false;
    }

    int width = frame.stFrameInfo.nWidth;
    int height = frame.stFrameInfo.nHeight;
    int pixelType = frame.stFrameInfo.enPixelType;

    std::vector<uint8_t> buffer(frame.stFrameInfo.nFrameLen);
    memcpy(buffer.data(), frame.pBufAddr, frame.stFrameInfo.nFrameLen);

    if (pixelType == PixelType_Gvsp_BayerGB8) {
        cv::Mat bayerImg(height, width, CV_8UC1, buffer.data());
        cv::cvtColor(bayerImg, image, cv::COLOR_BayerGR2BGR);
    } else if (pixelType == PixelType_Gvsp_Mono8) {
        cv::Mat grayImg(height, width, CV_8UC1, buffer.data());
        cv::cvtColor(grayImg, image, cv::COLOR_GRAY2BGR);
    } else {
        std::cerr << "Unsupported pixel type: " << pixelType << std::endl;
        MV_CC_FreeImageBuffer(camHandle, &frame);
        return false;
    }

    MV_CC_FreeImageBuffer(camHandle, &frame);
    return true;
}

void HikCamera::release() {
    if (!camHandle) return;

    MV_CC_StopGrabbing(camHandle);
    MV_CC_CloseDevice(camHandle);
    MV_CC_DestroyHandle(camHandle);
    camHandle = nullptr;
    connected = false;

    MV_CC_Finalize();
}