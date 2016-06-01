#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#include "VisualOdometry.h"
#include "PCLCloudSearch.h"
#include "Common.h"

using namespace std;
using namespace cv;

// http://programmers.stackexchange.com/questions/207577/when-comparing-floats-what-do-you-call-the-threshold-of-difference
#define FLT_THRESHOLD       1.192092896e-07F
#define DBL_THRESHOLD       2.2204460492503131e-016
#define THRESHOLD           FLT_THRESHOLD

bool almostEqual(double p1, double p2, double threshold)
{
    if (abs((double)(p1-p2)) < threshold)
        return true;
    else
        return false;
}

bool comparePoint3D(Point3d p1, Point3d p2)
{
    if (p1.x != p2.x)
        return p1.x > p2.x;
    else if (p1.y != p2.y)
        return  p1.y > p2.y;
    else
        return p1.z > p2.z;
}

bool equalPoint3D(Point3d p1, Point3d p2)
{
    if (almostEqual(p1.x, p2.x, THRESHOLD) &&
        almostEqual(p1.y, p2.y, THRESHOLD) &&
        almostEqual(p1.z, p2.z, THRESHOLD))
        return true;
    return false;
}

bool comparePoint2D(Point2d p1, Point2d p2)
{
    if (p1.x != p2.x)
        return p1.x > p2.x;
    else
        return p1.y > p2.y;
}

bool equalPoint2D(Point2d p1, Point2d p2)
{
    if (almostEqual(p1.x, p2.x, THRESHOLD) && almostEqual(p1.y, p2.y, THRESHOLD))
        return true;
    return false;
}

bool comparePair (pair<Point3d, Point2d> p1, pair<Point3d, Point2d> p2)
{
    return comparePoint3D(p1.first, p2.first);
}

bool equalPair (pair<Point3d, Point2d> p1, pair<Point3d, Point2d> p2)
{
    return equalPoint3D(p1.first, p2.first);
}



/* -------------------------------------------------------------------------------------------------------------------------------------------- */
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

//    Common com;
//    
//    char filepath[100] = "3DduplicateRemovalSample.txt";
//    
//    vector<Point2d> imageP;
//    vector<Point3d> worldP;
//    
//    com.readCsvTo3D2D(filepath, worldP, imageP);
//    
//    vector<pair<Point3d, Point2d> > pairOf3Dto2D;
//    
//    for (int i=0; i<worldP.size(); i++)
//        pairOf3Dto2D.push_back(make_pair(worldP[i], imageP[i]));
//    
//    std::sort(pairOf3Dto2D.begin(), pairOf3Dto2D.end(), comparePair);
//    auto unique_end = std::unique(pairOf3Dto2D.begin(), pairOf3Dto2D.end(), equalPair);
//    pairOf3Dto2D.erase(unique_end, pairOf3Dto2D.end());
//    
//    std::sort(worldP.begin(), worldP.end(), comparePoint3D);
//    auto unique_endd = std::unique(worldP.begin(), worldP.end(), equalPoint3D);
//    worldP.erase(unique_endd, worldP.end());
    
    return 0;
}
