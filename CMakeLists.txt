cmake_minimum_required(VERSION 3.5)
project(camera_yolov5_detection)

# 设置架构特定路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HIKVISION_LIB "/opt/MVS/lib/aarch64")
    message(STATUS "Building for aarch64 architecture")
else()
    set(HIKVISION_LIB "/opt/MVS/lib/64")
    message(STATUS "Building for x86_64 architecture")
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG " -g -O0")
set(CMAKE_C_FLAGS_DEBUG " -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE " -O3")
set(CMAKE_C_FLAGS_RELEASE " -O3")

# 查找OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found, please install OpenCV first")
else()
    message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
endif()

# 设置BPU库路径
set(DNN_PATH "/usr/include/dnn")
set(DNN_LIB_PATH "/usr/lib/")

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${DNN_PATH}
    ${OpenCV_INCLUDE_DIRS}
    /opt/MVS/include  # 海康头文件
)

link_directories(
    ${DNN_LIB_PATH}
    ${OpenCV_LIBRARY_DIRS}
    ${HIKVISION_LIB}  # 架构自适应路径
)

# 添加可执行文件
add_executable(camera_yolov5_detection
    src/main.cpp
    src/camera_yolo.cpp
    src/yolov5_detector.cpp
    src/serial_port.cpp
)

# 链接库（基本库）
target_link_libraries(camera_yolov5_detection
    ${OpenCV_LIBS}
    dnn
    pthread
    rt
    dl
    "${HIKVISION_LIB}/libMvCameraControl.so"  # 绝对路径链接
)


# 设置RPATH以便在运行时找到库
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)