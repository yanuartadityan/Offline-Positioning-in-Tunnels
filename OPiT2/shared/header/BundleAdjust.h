#ifndef __BUNDLE_ADJUST_HEADER__
#define __BUNDLE_ADJUST_HEADER__

#include <cvsba/cvsba.h>

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <iostream>

using namespace std;
using namespace cv;

class BundleAdjust
{
public:
    struct frameInfo
    {
        vector<Point2d> imagePoints;
        vector<Point3d> worldPoints;
        Mat             K;
        Mat             R;
        Mat             t;
        Mat             distortCoef;
    };

    BundleAdjust();

    // //remove duplicates
    // bool almostEqual(double p1, double p2, double threshold);
    // bool comparePoint3D(Point3d p1, Point3d p2);
    // bool equalPoint3D(Point3d p1, Point3d p2);
    // bool comparePoint2D(Point2d p1, Point2d p2);
    // bool equalPoint2D(Point2d p1, Point2d p2);
    // bool comparePair (pair<Point3d, Point2d> p1, pair<Point3d, Point2d> p2);
    // bool equalPair (pair<Point3d, Point2d> p1, pair<Point3d, Point2d> p2);
    // void pairFrom3Dto2D (vector<Point3d>, vector<Point2d>, vector<pair<Point3d, Point2d> > &);
    // void removeDuplicate(vector<Point3d> &worldPoints);

    void pushFrame (vector<Point2d> &, vector<Point3d> &, Mat &, Mat &, Mat &, Mat &);
    void eraseFirstFrame();
    void run();
    void prepareData(vector<Point3d> &, vector<vector<Point2d> > &, vector<vector<int> > &,
                     vector<Mat> &, vector<Mat> &, vector<Mat> &, vector<Mat> &);
    int  getWindowSize();
private:
    vector<frameInfo>   frameWindows;
    cvsba::Sba          sba;
    cvsba::Sba::Params  params;
};

#endif
