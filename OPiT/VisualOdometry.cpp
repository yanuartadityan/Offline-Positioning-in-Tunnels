#include "VisualOdometry.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <iostream>

using namespace std;
using namespace cv;

/*
    A class for implementing visual odometry.

    it is based on the visual odometry tutorial by David Scaramuzza.

    this particular version will use 2d-to-2d correspondences gathered

    by performing feature detection between two aligned images.

    by doing this, essential matrix could be derived and R|t matrix

    could be decomposed. It is used to estimate the camera pose
*/

// constructor, build the camera matrix
VO::VO()											// WE HAVE TO TAKE DISTORTION INTO ACCOUNT!
{																//							    Camera matrix
	cameraMatrix = Mat(3, 3, CV_64FC1, Scalar::all(0));			//								___		  ___
	cameraMatrix.at<double>(0, 0) = 1432;						// Focal length X				| fx  0  cx |
	cameraMatrix.at<double>(1, 1) = 1432;						// Focal length Y				| 0  fy  cy |
	cameraMatrix.at<double>(0, 2) = 640;						// Principal point X			| 0   0   1 |
	cameraMatrix.at<double>(1, 2) = 481;						// Principal point Y			---		  ---
	cameraMatrix.at<double>(2, 2) = 1.0;						// Just a 1 cause why not
}

void VO::visualodometry()
{
    // start the visual odometry
    
    // 1. load the first two images
    char filename1[100];
    char filename2[100];
    
    sprintf(filename1, "%simg_%05d.png", imagePath, 1);
    sprintf(filename2, "%simg_%05d.png", imagePath, 2);
    
    Mat firstImage = imread(filename1);
    Mat secondImage = imread(filename2);
    
    // 2. work on grayscale images only
    cvtColor(firstImage, firstImage, COLOR_BGR2GRAY);
    cvtColor(secondImage, secondImage, COLOR_BGR2GRAY);
    
    // 3. feature detection
    vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
    featureDetection(firstImage, points1, VO_METHOD_FAST);
    
}

void VO::featureDetection(cv::Mat img, std::vector<cv::Point2f>& points, int vo_method)
{
    vector<KeyPoint> keypoints;
    
    cv::namedWindow("detected features", WINDOW_NORMAL);
    
    if (vo_method == VO_METHOD_FAST)
    {
        cv::FAST(img, keypoints, fast_threshold, nonMaxSuppression);
        cv::KeyPoint::convert(keypoints, points, vector<int>());
    }
    
    Mat output;
    drawKeypoints(img, keypoints, output, Scalar(5, 5, 255), DrawMatchesFlags::DEFAULT);
    
    imshow("output", output);
    
    cv::waitKey(0);
}

void VO::initParameter()
{
    iterationsCount = 100;
    reprojectionError = 8;
    confidence = 0.99;
    method = cv::RANSAC;
    
    // fast
    fast_threshold = 20;
    nonMaxSuppression = true;
    
    // surf
    min_hessian = 1000;
    
    // sift
}

void VO::setImagePath(char *pathname)
{
    strcpy(imagePath, pathname);
}
