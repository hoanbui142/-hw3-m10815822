#include <iostream>
#include <fstream>

#include "hw3-m10815822.hpp"

void CameraCalibration::readCoords()
{
    unit_square.push_back(cv::Point2f(0,0));
    unit_square.push_back(cv::Point2f(0,50));
    unit_square.push_back(cv::Point2f(50,50));
    unit_square.push_back(cv::Point2f(50,0));
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

void CameraCalibration::findK()
{
    //cv::Mat src_img = cv::imread("CalibrationIMG.bmp");
    // Find Homography of Square One, Two, Three
    cv::Mat h1 = cv::findHomography(square_one, unit_square);
    cv::Mat h2 = cv::findHomography(square_two, unit_square);
    cv::Mat h3 = cv::findHomography(square_three, unit_square);
    std::cout << "h1 = \n"<< h1 << std::endl;
    double h11 = h1.at<double>(0,0);
    double h12 = h1.at<double>(0,1);
    double h13 = h1.at<double>(0,2);
    double h21 = h1.at<double>(1,0);
    double h22 = h1.at<double>(1,1);
    double h23 = h1.at<double>(1,2);
    double h31 = h1.at<double>(2,0);
    double h32 = h1.at<double>(2,1);
    double h33 = h1.at<double>(2,2);
    std::cout << "h11 = \n"<< h11 << std::endl;
    std::cout << "h12 = \n"<< h12 << std::endl;
    std::cout << "h13 = \n"<< h13 << std::endl;
    std::cout << "h21 = \n"<< h21 << std::endl;
    std::cout << "h22 = \n"<< h22 << std::endl;
    std::cout << "h23 = \n"<< h23 << std::endl;
    std::cout << "h31 = \n"<< h31 << std::endl;
    std::cout << "h32 = \n"<< h32 << std::endl;
    std::cout << "h33 = \n"<< h33 << std::endl;

    // h1 & h2 of Homography 1 - square_one --> two equations (constraints)
    double A11 = h1.at<double>(0,0) * h1.at<double>(1,0);
    double A12 = (h1.at<double>(0,0) * h1.at<double>(1,1)) + (h1.at<double>(0,1) * h1.at<double>(1,0));
    double A13 = (h1.at<double>(0,0) * h1.at<double>(1,2)) + (h1.at<double>(0,2) * h1.at<double>(1,0));
    double A14 = h1.at<double>(0,1) * h1.at<double>(1,1);
    double A15 = (h1.at<double>(0,1) * h1.at<double>(1,2)) + (h1.at<double>(0,2) * h1.at<double>(1,1));
    double A16 = h1.at<double>(0,2) * h1.at<double>(1,2);
    double A21 = std::pow((h1.at<double>(0,0)),2) - std::pow((h1.at<double>(1,0)),2);
    double A22 = 2 * ((h1.at<double>(0,0) * h1.at<double>(0,1)) - (h1.at<double>(1,0) * h1.at<double>(1,1)));
    double A23 = 2 * ((h1.at<double>(0,0) * h1.at<double>(0,2)) - (h1.at<double>(1,0) * h1.at<double>(1,2)));
    double A24 = std::pow((h1.at<double>(0,1)),2) - std::pow((h1.at<double>(1,1)),2);
    double A25 = 2 * ((h1.at<double>(0,1) * h1.at<double>(0,2)) - (h1.at<double>(1,1) * h1.at<double>(1,2)));
    double A26 = std::pow((h1.at<double>(0,2)),2) - std::pow((h1.at<double>(1,2)),2);

    // h1 & h2 of Homography 2 - square_two --> two equations (constraints)
    double A31 = h2.at<double>(0,0) * h2.at<double>(1,0);
    double A32 = (h2.at<double>(0,0) * h2.at<double>(1,1)) + (h2.at<double>(0,1) * h2.at<double>(1,0));
    double A33 = (h2.at<double>(0,0) * h2.at<double>(1,2)) + (h2.at<double>(0,2) * h2.at<double>(1,0));
    double A34 = h2.at<double>(0,1) * h2.at<double>(1,1);
    double A35 = (h2.at<double>(0,1) * h2.at<double>(1,2)) + (h2.at<double>(0,2) * h2.at<double>(1,1));
    double A36 = h2.at<double>(0,2) * h2.at<double>(1,2);
    double A41 = std::pow((h2.at<double>(0,0)),2) - std::pow((h2.at<double>(1,0)),2);
    double A42 = 2 * ((h2.at<double>(0,0) * h2.at<double>(0,1)) - (h2.at<double>(1,0) * h2.at<double>(1,1)));
    double A43 = 2 * ((h2.at<double>(0,0) * h2.at<double>(0,2)) - (h2.at<double>(1,0) * h2.at<double>(1,2)));
    double A44 = std::pow((h2.at<double>(0,1)),2) - std::pow((h2.at<double>(1,1)),2);
    double A45 = 2 * ((h2.at<double>(0,1) * h2.at<double>(0,2)) - (h2.at<double>(1,1) * h2.at<double>(1,2)));
    double A46 = std::pow((h2.at<double>(0,2)),2) - std::pow((h2.at<double>(1,2)),2);

     // h1 & h2 of Homography 3 - square_three --> two equations (constraints)
    double A51 = h3.at<double>(0,0) * h3.at<double>(1,0);
    double A52 = (h3.at<double>(0,0) * h3.at<double>(1,1)) + (h3.at<double>(0,1) * h3.at<double>(1,0));
    double A53 = (h3.at<double>(0,0) * h3.at<double>(1,2)) + (h3.at<double>(0,2) * h3.at<double>(1,0));
    double A54 = h3.at<double>(0,1) * h3.at<double>(1,1);
    double A55 = (h3.at<double>(0,1) * h3.at<double>(1,2)) + (h3.at<double>(0,2) * h3.at<double>(1,1));
    double A56 = h3.at<double>(0,2) * h3.at<double>(1,2);
    double A61 = std::pow((h3.at<double>(0,0)),2) - std::pow((h3.at<double>(1,0)),2);
    double A62 = 2 * ((h3.at<double>(0,0) * h3.at<double>(0,1)) - (h3.at<double>(1,0) * h3.at<double>(1,1)));
    double A63 = 2 * ((h3.at<double>(0,0) * h3.at<double>(0,2)) - (h3.at<double>(1,0) * h3.at<double>(1,2)));
    double A64 = std::pow((h3.at<double>(0,1)),2) - std::pow((h3.at<double>(1,1)),2);
    double A65 = 2 * ((h3.at<double>(0,1) * h3.at<double>(0,2)) - (h3.at<double>(1,1) * h3.at<double>(1,2)));
    double A66 = std::pow((h3.at<double>(0,2)),2) - std::pow((h3.at<double>(1,2)),2);
    
    //cv::Mat A122 = (h1.col(0).row(0) * h1.col(1).row(1)) + (h1.col(0).row(1) * h1.col(1).row(0));
    //cv::Mat A13 = (h1.col(0).row(0) * h1.col(1).row(2)) + (h1.col(0).row(2) * h1.col(1).row(0));
    //cv::Mat A14 = h1.col(0).row(1) * h1.col(1).row(1);
    //cv::Mat A15 = (h1.col(0).row(1) * h1.col(1).row(2)) + (h1.col(0).row(2) * h1.col(1).row(1));
    //cv::Mat A16 = h1.col(0).row(2) * h1.col(1).row(2);
    //cv::Mat A21 = std::pow((h1.col(0).row(0)),2.) - std::pow((h1.col(1).row(0)),2);
    //std::cout << A12 << std::endl;
    //std::cout << A21 << std::endl;
}

void CameraCalibration::findRt()
{
    //cv::Mat M(100, 100, CV_64F);
    //std::cout << M.at<double>(100,100);
    short mydata[]={1, 2, 1, 3, 4, 5, 6, 7, 8};
    cv::Mat H(3,3,CV_16SC1,mydata);
    std::cout << "test = " << H << std::endl;
    std::cout << "H11 =" << H.at<short>(1,2) << std::endl;
}