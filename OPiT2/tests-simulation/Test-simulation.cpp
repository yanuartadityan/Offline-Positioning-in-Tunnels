//OPENCV
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

//OUR STUFF
#include "FeatureDetection.h"
#include "Calibration.h"
#include "PnPSolver.h"
#include "PCLCloudSearch.h"
#include "Common.h"

#include <iostream>
#include <fstream>
#include <numeric>

#define STARTIDX                433             // frame start, it could be the previous frame or the latter one as long it finds a hit with LUT

using namespace std;
using namespace cv;
using namespace std::chrono;

int main(int argc, char* argv[])
{
    Calibration cal;
    PnPSolver pnp;
    
    // set the world points
    vector <Point3d> worldPoints;
    //worldPoints.push_back(Point3d(143427.594, 6394362.013, 38.361));
    //worldPoints.push_back(Point3d(143436.438, 6394359.361, 38.317));
    //worldPoints.push_back(Point3d(143433.614, 6394361.699, 40.287));
    worldPoints.push_back(Point3d(143436.267, 6394357.480, 34.037));
    worldPoints.push_back(Point3d(143427.090, 6394362.320, 37.439));
    //worldPoints.push_back(Point3d(143436.859, 6394359.122, 37.570));
    worldPoints.push_back(Point3d(143427.055, 6394361.512, 33.561));
    //worldPoints.push_back(Point3d(143438.340, 6394364.051, 37.272));
    //worldPoints.push_back(Point3d(143438.351, 6394364.068, 36.756));
    //worldPoints.push_back(Point3d(143438.344, 6394364.076, 36.411));
    //worldPoints.push_back(Point3d(143438.345, 6394364.092, 35.815));
    worldPoints.push_back(Point3d(143427.073, 6394362.323, 35.505));
    
    // set the image points
    vector <Point2d> imagePoints;
    //imagePoints.push_back(Point2d(239.8, 207.1));
    //imagePoints.push_back(Point2d(834.7, 174.6));
    //imagePoints.push_back(Point2d(601.5, 95.00)); //
    imagePoints.push_back(Point2d(887.8, 453.1)); //
    imagePoints.push_back(Point2d(211.3, 268.4)); //
    //imagePoints.push_back(Point2d(876.0, 226.6));
    imagePoints.push_back(Point2d(211.2, 510.4)); //
    //imagePoints.push_back(Point2d(831.8, 294.6));
    //imagePoints.push_back(Point2d(834.1, 321.7));
    //imagePoints.push_back(Point2d(835.2, 341.2));
    //imagePoints.push_back(Point2d(836.4, 365.5));
    imagePoints.push_back(Point2d(206.1, 401.5));
    
    // best with 4-5-7-12
    
    // with RANSAC
    pnp.setPnPParam (1000, 5, 0.99);
    pnp.setImagePoints(imagePoints);
    pnp.setWorldPoints(worldPoints);
    pnp.run(1);

    Mat rVec3x1, tVec3x1, rVec3x3;
    solvePnP(worldPoints, imagePoints, cal.getCameraMatrix(), cal.getDistortionCoeffs(), rVec3x1, tVec3x1, false, CV_P3P);
    Rodrigues(rVec3x1, rVec3x3);
    Mat cameraPosition = -(rVec3x3.t()) * tVec3x1;
    cout << cameraPosition << endl;
    
    return 0;
}
