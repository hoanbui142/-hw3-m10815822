#include <iostream>
#include <fstream>

#include "hw3-m10815822.hpp"

void CameraCalibration::readCoords()
{
    /*unit_square.push_back(cv::Point2f(0,0));
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
    square_three.push_back(cv::Point2f(401,288));*/
    unit_square.push_back(cv::Point2f(0,0));
    unit_square.push_back(cv::Point2f(0,1));
    unit_square.push_back(cv::Point2f(1,1));
    unit_square.push_back(cv::Point2f(1,0));
    square_one.push_back(cv::Point2f(152,149));
    square_one.push_back(cv::Point2f(218,413));
    square_one.push_back(cv::Point2f(490,332));
    square_one.push_back(cv::Point2f(482,77));
    square_two.push_back(cv::Point2f(596,84));
    square_two.push_back(cv::Point2f(596,334));
    square_two.push_back(cv::Point2f(838,458));
    square_two.push_back(cv::Point2f(898,195));
    square_three.push_back(cv::Point2f(490,387));
    square_three.push_back(cv::Point2f(343,602));
    square_three.push_back(cv::Point2f(689,722));
    square_three.push_back(cv::Point2f(780,465));
}

void CameraCalibration::findK()
{
    // Find Homography of Square One, Two, Three
    cv::Mat H1 = cv::findHomography(unit_square,square_one);
    cv::Mat H2 = cv::findHomography(unit_square,square_two);
    cv::Mat H3 = cv::findHomography(unit_square,square_three);
    //std::cout << "h1 = \n"<< h1 << std::endl;
    //std::cout << "h2 = \n"<< h2 << std::endl;
    //std::cout << "h3 = \n"<< h3 << std::endl;
    /*double h11 = h1.at<double>(0,0);
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
    std::cout << "h33 = \n"<< h33 << std::endl;*/

    // h1 & h2 of Homography 1 - square_one --> two equations (constraints)
    double A11 = H1.at<double>(0,0) * H1.at<double>(0,1);
    double A12 = (H1.at<double>(0,0) * H1.at<double>(1,1)) + (H1.at<double>(1,0) * H1.at<double>(0,1));
    double A13 = (H1.at<double>(0,0) * H1.at<double>(2,1)) + (H1.at<double>(2,0) * H1.at<double>(0,1));
    double A14 = H1.at<double>(1,0) * H1.at<double>(1,1);
    double A15 = (H1.at<double>(1,0) * H1.at<double>(2,1)) + (H1.at<double>(2,0) * H1.at<double>(1,1));
    double A16 = H1.at<double>(2,0) * H1.at<double>(2,1);
    double A21 = std::pow((H1.at<double>(0,0)),2.) - std::pow((H1.at<double>(0,1)),2.);
    double A22 = 2 * ((H1.at<double>(0,0) * H1.at<double>(1,0)) - (H1.at<double>(0,1) * H1.at<double>(1,1)));
    double A23 = 2 * ((H1.at<double>(0,0) * H1.at<double>(2,0)) - (H1.at<double>(0,1) * H1.at<double>(2,1)));
    double A24 = std::pow((H1.at<double>(1,0)),2.) - std::pow((H1.at<double>(1,1)),2.);
    double A25 = 2 * ((H1.at<double>(1,0) * H1.at<double>(2,0)) - (H1.at<double>(1,1) * H1.at<double>(2,1)));
    double A26 = std::pow((H1.at<double>(2,0)),2.) - std::pow((H1.at<double>(2,1)),2.);
    
    // h1 & h2 of Homography 2 - square_two --> two equations (constraints)
    double A31 = H2.at<double>(0,0) * H2.at<double>(0,1);
    double A32 = (H2.at<double>(0,0) * H2.at<double>(1,1)) + (H2.at<double>(1,0) * H2.at<double>(0,1));
    double A33 = (H2.at<double>(0,0) * H2.at<double>(2,1)) + (H2.at<double>(2,0) * H2.at<double>(0,1));
    double A34 = H2.at<double>(1,0) * H2.at<double>(1,1);
    double A35 = (H2.at<double>(1,0) * H2.at<double>(2,1)) + (H2.at<double>(2,0) * H2.at<double>(1,1));
    double A36 = H2.at<double>(2,0) * H2.at<double>(2,1);
    double A41 = std::pow((H2.at<double>(0,0)),2.) - std::pow((H2.at<double>(0,1)),2.);
    double A42 = 2 * ((H2.at<double>(0,0) * H2.at<double>(1,0)) - (H2.at<double>(0,1) * H2.at<double>(1,1)));
    double A43 = 2 * ((H2.at<double>(0,0) * H2.at<double>(2,0)) - (H2.at<double>(0,1) * H2.at<double>(2,1)));
    double A44 = std::pow((H2.at<double>(1,0)),2.) - std::pow((H2.at<double>(1,1)),2.);
    double A45 = 2 * ((H2.at<double>(1,0) * H2.at<double>(2,0)) - (H2.at<double>(1,1) * H2.at<double>(2,1)));
    double A46 = std::pow((H2.at<double>(2,0)),2.) - std::pow((H2.at<double>(2,1)),2.);

     // h1 & h2 of Homography 3 - square_three --> two equations (constraints)
    double A51 = H3.at<double>(0,0) * H3.at<double>(0,1);
    double A52 = (H3.at<double>(0,0) * H3.at<double>(1,1)) + (H3.at<double>(1,0) * H3.at<double>(0,1));
    double A53 = (H3.at<double>(0,0) * H3.at<double>(2,1)) + (H3.at<double>(2,0) * H3.at<double>(0,1));
    double A54 = H3.at<double>(1,0) * H3.at<double>(1,1);
    double A55 = (H3.at<double>(1,0) * H3.at<double>(2,1)) + (H3.at<double>(2,0) * H3.at<double>(1,1));
    double A56 = H3.at<double>(2,0) * H3.at<double>(2,1);
    double A61 = std::pow((H3.at<double>(0,0)),2.) - std::pow((H3.at<double>(0,1)),2.);
    double A62 = 2 * ((H3.at<double>(0,0) * H3.at<double>(1,0)) - (H3.at<double>(0,1) * H3.at<double>(1,1)));
    double A63 = 2 * ((H3.at<double>(0,0) * H3.at<double>(2,0)) - (H3.at<double>(0,1) * H3.at<double>(2,1)));
    double A64 = std::pow((H3.at<double>(1,0)),2.) - std::pow((H3.at<double>(1,1)),2.);
    double A65 = 2 * ((H3.at<double>(1,0) * H3.at<double>(2,0)) - (H3.at<double>(1,1) * H3.at<double>(2,1)));
    double A66 = std::pow((H3.at<double>(2,0)),2.) - std::pow((H3.at<double>(2,1)),2.);
    
    // Construct the Matrix form A
    double dataA[] = {A11, A12, A13, A14, A15, A16, 
                      A21, A22, A23, A24, A25, A26, 
                      A31, A32, A33, A34, A35, A36, 
                      A41, A42, A43, A44, A45, A46, 
                      A51, A52, A53, A54, A55, A56, 
                      A61, A62, A63, A64, A65, A66};
    cv::Mat A = cv::Mat(6,6,CV_64FC1,dataA);
    //std::cout << "A = \n" << A << std::endl;

    //Determine w
    cv::Mat S, U, V;
    cv::SVD::compute(A, S, U, V, cv::SVD::FULL_UV);
    //std::cout << "V" << std::endl << V << std::endl << std::endl;
    cv::Mat Vt = V.t();
    //std::cout << "V" << std::endl << Vt << std::endl << std::endl;
    double w11 = Vt.at<double>(0,5);
    double w12 = Vt.at<double>(1,5);
    double w13 = Vt.at<double>(2,5);
    double w21 = Vt.at<double>(1,5);
    double w22 = Vt.at<double>(3,5);
    double w23 = Vt.at<double>(4,5);
    double w31 = Vt.at<double>(2,5);
    double w32 = Vt.at<double>(4,5);
    double w33 = Vt.at<double>(5,5);
    double dataw[] = {w11, w12, w13,
                      w21, w22, w23,
                      w31, w32, w33};
    cv::Mat w = cv::Mat(3,3,CV_64FC1,dataw);                       
    //std::cout << "w = \n" << w << std::endl;

    //inverse & normalize w
    cv::Mat inw = w.inv();
    //std::cout << "inw = \n" << inw << std::endl;
    cv::Mat inw_normalize = inw/inw.row(2).col(2);
    //std::cout << "inw = \n" << inw_normalize << std::endl;

    //Determine K
    double c = inw_normalize.at<double>(0,2);
    double e = inw_normalize.at<double>(1,2);
    double d = sqrt((inw_normalize.at<double>(1,1)) - std::pow(e,2.));
    double b = ((inw_normalize.at<double>(0,1)) - c * e)/ d;
    double a = sqrt((inw_normalize.at<double>(0,0)) - std::pow(b,2.) - std::pow(c,2.));
    double dataK[] = {a, b, c,
                      0., d, e,
                      0., 0., 1.};
    cv::Mat K = cv::Mat(3,3,CV_64FC1,dataK);
    std::cout << "K = \n" << K << std::endl << std::endl;
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