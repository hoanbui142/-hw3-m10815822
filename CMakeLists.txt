cmake_minimum_required(VERSION 3.10)

project(cv-hw3-m10815822)

set(OpenCV_DIR "C:\\opencv\\build")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(hw3-m10815822
    src/hw3-m10815822.cpp
    src/camera-calibration.cpp
)

target_link_libraries(hw3-m10815822 ${OpenCV_LIBS})
