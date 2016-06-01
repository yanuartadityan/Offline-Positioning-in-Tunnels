#include "Common.h"
#include "BundleAdjust.h"

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <cvsba/cvsba.h>

#include <iostream>

using namespace std;
using namespace cv;

//#define FLT_THRESHOLD   1.192092896e-07F
#define FLT_THRESHOLD   0.05            // 1 cm tolerancy

/* ---------------------------------------------------------------------------------------------------------
    bunch of methods for removing the duplicates in 3D-to-2D correspondences
    so, given N pair of 3D to 2D correspondences, in vector of Point3d and Point2d
    create a vector pair of <Point3d, Point2d> and then filter them down using std::sort and std::unique
   ---------------------------------------------------------------------------------------------------------*/
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
    if (almostEqual(p1.x, p2.x, FLT_THRESHOLD) &&
        almostEqual(p1.y, p2.y, FLT_THRESHOLD) &&
        almostEqual(p1.z, p2.z, FLT_THRESHOLD))
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
    if (almostEqual(p1.x, p2.x, FLT_THRESHOLD) && almostEqual(p1.y, p2.y, FLT_THRESHOLD))
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

void pairFrom3Dto2D (vector<Point3d> worldPoints, vector<Point2d> imagePoints, vector<pair<Point3d, Point2d> > &pairOf3Dto2D)
{
    for (int i=0; i<worldPoints.size(); i++)
        pairOf3Dto2D.push_back(make_pair(worldPoints[i], imagePoints[i]));
}

/* ------------------------------------------------------------------------------------------------ */


BundleAdjust::BundleAdjust()
{
    this->frameWindows.clear();

    // init the param, change as we desire
    this->params.type = cvsba::Sba::MOTIONSTRUCTURE;
    this->params.iterations = 150;
    this->params.minError = 1e-10;
    this->params.fixedIntrinsics = 5;           // focal x, focal y, principal x, principal y, skew
    this->params.fixedDistortion = 5;           // 5, as mentioned in caltech camera calibration module
    this->params.verbose = true;
    this->sba.setParams(params);
}

void BundleAdjust::pushFrame(vector<Point2d> image2D, vector<Point3d> image3D, Mat K, Mat Rvec, Mat tvec, Mat dist)
{
    frameInfo temp;
    temp.imagePoints    = image2D;
    temp.worldPoints    = image3D;
    temp.K              = K;
    temp.R              = Rvec;
    temp.t              = tvec;
    temp.distortCoef    = dist;

    this->frameWindows.push_back (temp);
}

void BundleAdjust::eraseFirstFrame()
{
    this->frameWindows.erase(this->frameWindows.begin());
}

int BundleAdjust::getWindowSize()
{
    return BundleAdjust::frameWindows.size();
}

void BundleAdjust::prepareData(vector<Point3d> &points3D,
                               vector<vector<Point2d> > &pointsImg,
                               vector<vector<int> > &visibility,
                               vector<Mat> &cameraMatrix,
                               vector<Mat> &R,
                               vector<Mat> &T,
                               vector<Mat> &distCoeffs)
{
    // create a list of 3D and 2D from all frames inside window
    cout << "  bundle adjustment..." << endl;
    for (int i=0; i<getWindowSize(); i++)
    {
        // prepare all 3D points
        points3D.insert(std::end(points3D), std::begin(frameWindows[i].worldPoints), std::end(frameWindows[i].worldPoints));
        cout << "    window-" << i << "-th with " << frameWindows[i].worldPoints.size() << " 3D points" << endl;
    }
    
    // check duplicates of 3D points to some very small threshold and erase the duplicate
//    std::sort(points3D.begin(), points3D.end(), comparePoint3D);
//    auto unique_end = std::unique(points3D.begin(), points3D.end(), equalPoint3D);
//    points3D.erase(unique_end, points3D.end());

    // allocate 2D vectors
    visibility.resize(getWindowSize());
    pointsImg.resize(getWindowSize());
    for (int i=0; i<getWindowSize(); i++)
    {
        visibility[i].resize(points3D.size());
        pointsImg[i].resize(points3D.size());
        
        for (int j=0; j<points3D.size(); j++)
        {
            pointsImg[i][j].x = std::numeric_limits<float>::quiet_NaN();
            pointsImg[i][j].y = std::numeric_limits<float>::quiet_NaN();
        }
    }
    
    
    
    // check visibility by checking each elements inside points3D and returns visibility mask and corresponding
    // 2d feature points (for each frame)
    int visCount = 0;
    int winSize = getWindowSize();
    for (int i=0; i<points3D.size(); i++)
    {
        for (int j=0; j<winSize; j++)
        {
            for (int k=0; k<frameWindows[j].worldPoints.size(); k++)
            {
                if (almostEqual(frameWindows[j].worldPoints[k].x, points3D[i].x, FLT_THRESHOLD) &&
                    almostEqual(frameWindows[j].worldPoints[k].y, points3D[i].y, FLT_THRESHOLD) &&
                    almostEqual(frameWindows[j].worldPoints[k].z, points3D[i].z, FLT_THRESHOLD))
                {
                    visibility[j][i] = 1;
                    pointsImg[j][i] = Point2d(frameWindows[j].imagePoints[k].x,frameWindows[j].imagePoints[k].y);
                    visCount++;
                }
            }
        }
    }

    cout << "  visible 3D points: " << visCount << endl;

    // append camera matrix, R, t and distCoeffs
    Mat rTemp;
    for (int i=0; i<getWindowSize(); i++)
    {
        // convert from 3x3 matrix to Rodrigues rotation matrix
        Rodrigues(frameWindows[i].R, rTemp);
    
        R.push_back(rTemp);
        T.push_back(frameWindows[i].t);
        cameraMatrix.push_back(frameWindows[i].K);
        distCoeffs.push_back((Mat1d(5,1) << 0, 0, 0, 0, 0));
        
        // debugging
        cout << "    R for frame-" << i << endl << frameWindows[i].R << endl;
        cout << "    rodriguesed to" << endl << rTemp << endl;
        cout << "    K for frame-" << i << endl << frameWindows[i].K << endl;
    }
}

void BundleAdjust::run()
{
    TermCriteria                    criteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 150, 1e-10);
    vector<Point3d>                 points3D;
    vector<vector<cv::Point2d> >    pointsImg;
    vector<vector<int> >            visibility;
    vector<Mat>                     cameraMatrix;
    vector<Mat>                     distCoeffs, R, T;

    // prepare the data for the bundle
    prepareData(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);

    // run the sparse bundle adjustment for N-window size
    double repError = sba.run(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
    
    cout << "  reprojection error after bundle adjustment: " << repError << endl;
}
