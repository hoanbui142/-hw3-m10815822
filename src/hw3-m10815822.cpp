#include "hw3-m10815822.hpp"

int main(int argc, char const *argv[])
{
    CameraCalibration calibration;
    calibration.readCoords();
    calibration.findK();
    
    return 0;
}