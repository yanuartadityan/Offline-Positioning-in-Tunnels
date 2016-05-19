#ifndef __COMMON_H_INCLUDED_
#define __COMMON_H_INCLUDED_

#include <iostream>
#include <chrono>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace std::chrono;
using namespace cv;

class Common
{
public:
    //constructor & destructor
    Common();
    ~Common();

    //timer
    void startTimer();
    void reportTimer();

    //logging
    static void createDir(const string dirname);

    //preparemap
    void prepareMap (char* mapCoordinateFile, char* mapKeypointsFile, vector<Point2d> *tunnel2Dx, vector<Point3d> *tunnel3Dx, Mat *tunnelDescriptor);
private:
    high_resolution_clock::time_point t1, t2;
};

#endif
