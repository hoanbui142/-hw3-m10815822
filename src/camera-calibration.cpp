#include <iostream>
#include <fstream>

#include "hw3-m10815822.hpp"

void CameraCalibration::readCoords()
{
    unit_square.push_back(cv::Point2f(0,0));
    unit_square.push_back(cv::Point2f(0,1));
    unit_square.push_back(cv::Point2f(1,1));
    unit_square.push_back(cv::Point2f(1,0));
    square_one.push_back(cv::Point2f(127,60));
    square_one.push_back(cv::Point2f(205,382));
    square_one.push_back(cv::Point2f(401,288));
    square_one.push_back(cv::Point2f(402,57));
    square_two.push_back(cv::Point2f(402,57));
    square_two.push_back(cv::Point2f(401,288));
    square_two.push_back(cv::Point2f(605,375));
    square_two.push_back(cv::Point2f(686,60));
    square_three.push_back(cv::Point2f(205,382));
    square_three.push_back(cv::Point2f(422,577));
    square_three.push_back(cv::Point2f(605,375));
    square_three.push_back(cv::Point2f(401,288));
}