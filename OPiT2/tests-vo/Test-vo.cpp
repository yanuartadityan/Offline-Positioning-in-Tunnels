#include <iostream>
#include <fstream>
#include "VisualOdometry.h"

using namespace std;

int main ()
{
    #ifndef KITTI_DATASET
       char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/volvo/out0/";
    #else
       char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/kitti-odometry/dataset/sequences/00/image_0/";
    #endif

    // VISUAL ODOMETRY
    VO vodometry;
    vodometry.initParameter();
    vodometry.setImagePath(pathname);
    vodometry.visualodometry();

    return 0;
}
