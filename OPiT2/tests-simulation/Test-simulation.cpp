//OPENCV
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//OUR STUFF
#include "FeatureDetection.h"
#include "Calibration.h"
#include "PnPSolver.h"
#include "PointProjection.h"
#include "PCLCloudSearch.h"
#include "Reprojection.h"

#include <iostream>
#include <fstream>
#include <numeric>
#include <thread>
#include <future>
#include <chrono>
#include <mutex>

using namespace std;

int main (int argc, char *argv[])
{
    // load the point cloud
    // setup the initial correspondences and store them in vocabulary (LUT)
    // load the first image (current)
    // feature detection on the current
    // feature matching current vs LUT
    // perform Lowe's criterion matching
    // push the good matches into the solvePnP class
    // solving the cam and obtained camera matrices
    // reproject features into point clouds
    // obtain average reprojection errors
    // reject bad 3d points with reprojection errros greater than the average
    // push the result to the LUT
    // do the while loop
        // load next image, set the previous current as (prev), set the next as new current
        // feature detection on the current
        // feature matching current vs prev, and get a good matches
        // feature matching good matches vs LUT
        // perform Lowe's criterion matching
        // push the good matches into the solvePnP class
        // solving the cam and obtained camera matrices
        // reproject 
    
    return 0;
}
