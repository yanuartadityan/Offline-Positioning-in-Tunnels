#include <iostream>
#include <fstream>

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#include "VisualOdometry.h"
#include "PCLCloudSearch.h"

using namespace std;
using namespace cv;

int main ()
{
    #ifndef KITTI_DATASET
       char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/volvo/out0/";
    #else
       char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/kitti-odometry/dataset/sequences/00/image_0/";
    #endif

    // VISUAL ODOMETRY
    // VO vodometry;
    // vodometry.initParameter();
    // vodometry.setImagePath(pathname);
    // vodometry.visualodometry();

    Point3d origin(5, 4, 3);
    Point3d sp(8, 7, 6);

    cout << origin << endl << sp << endl;

    double fu = cv::norm(sp-origin);
    cout << "fu is " << fu << endl;

    return 0;
}
