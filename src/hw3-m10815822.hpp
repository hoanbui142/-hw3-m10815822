#pragma once

#include <map>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

struct Coord3dpoint
{
    float x;
    float y;
    float z;
    //float w = 1;
}
class CameraCalibration
{
public:
    std::vector<cv::Point2f> unit_square;
    std::vector<cv::Point2f> square_one;
    std::vector<cv::Point2f> square_two;
    std::vector<cv::Point2f> square_three;
    void readCoords();
    void findK();
    void findRt();
    void findCameraPosition();
};