#include "PnPSolver.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <iostream>

using namespace cv;
using namespace std;

/*
	solvePnP defined here: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html


*/

int PnPSolver::foo()
{
	//MEASURE THE TIME
	int64 now, then;
	double elapsedSeconds, ticksPerSecond = cvGetTickFrequency()*1.0e6;

	then = cvGetTickCount();



	//2D image
	vector<Point2f> points1;
	points1.push_back(Point2d(384.3331f, 162.23618f));
	points1.push_back(Point2d(385.27521f, 135.21503f));
	points1.push_back(Point2d(409.36746f, 139.30315f));
	points1.push_back(Point2d(407.43854f, 165.64435f));

	//Real object points set
	//vector<Point3f> object;
	//object.push_back(Point3d(-88.0f, 88.0f, 0));
	//object.push_back(Point3d(-88.0f, -88.0f, 0));
	//object.push_back(Point3d(88.0f, -88.0f, 0));
	//object.push_back(Point3d(88.0f, 88.0f, 0));

	// 3D scene
	vector<Point3f> object;
	object.push_back(Point3d(143432.108, 6394362.287, 39.617));
	object.push_back(Point3d(143430.318, 6394363.127, 39.797));
	object.push_back(Point3d(143427.048, 6394361.520, 33.577));
	object.push_back(Point3d(143432.948, 6394364.927, 37.656));
	/*
	object.push_back(Point3d(143436.316, 6394359.472, 38.452));
	object.push_back(Point3d(143427.658, 6394362.027, 38.376));
	object.push_back(Point3d(143468.983, 6394441.362, 36.431));
	object.push_back(Point3d(143468.953, 6394441.302, 37.281));
	object.push_back(Point3d(143462.114, 6394450.099, 38.451));
	object.push_back(Point3d(143467.283, 6394451.589, 38.711));
	object.push_back(Point3d(143466.173, 6394449.469, 38.671));
	object.push_back(Point3d(143469.613, 6394456.418, 38.800));
	*/
															//								Camera matrix
	Mat cam_matrix = Mat(3, 3, CV_64FC1, Scalar::all(0));	//								___		  ___
	cam_matrix.at<double>(0, 0) = 1432;						// Focal length X				| fx  0  cx |
	cam_matrix.at<double>(1, 1) = 1432;						// Focal length Y				| 0  fy  cy |
	cam_matrix.at<double>(0, 2) = 640;						// Principal point X			| 0   0   0 |
	cam_matrix.at<double>(1, 2) = 481;						// Principal point Y			---		  ---
	cam_matrix.at<double>(2, 2) = 1.0;						// Just a 1 cause why not

	now = cvGetTickCount();
	elapsedSeconds = (double)(now - then) / ticksPerSecond;

	cout << "Initializtion took " << elapsedSeconds << " seconds" << endl;

	//PnP
	Mat rVecIter, tVecIter;
	Mat rVecP3P, tVecP3P;

	// Keep track of time
	then = cvGetTickCount();

	solvePnP(Mat(object), Mat(points1), cam_matrix, Mat(), rVecIter, tVecIter, false, CV_ITERATIVE);
	
	// Calculate time>
	now = cvGetTickCount();
	elapsedSeconds = (double)(now - then) / ticksPerSecond;
	cout << "PnP (ITERATIVE) took " << elapsedSeconds << " seconds" << endl;
	then = cvGetTickCount();

	// P3P requires exactly 4 points in both object and scene
	solvePnP(Mat(object), Mat(points1), cam_matrix, Mat(), rVecP3P, tVecP3P, false, CV_P3P);
	
	// Calculate time
	now = cvGetTickCount();
	elapsedSeconds = (double)(now - then) / ticksPerSecond;
	cout << "PnP (P3P) took " << elapsedSeconds << " seconds" << endl;


	//Print result
	cout << "Iterative: " << endl;
	cout << " rvec1 " << endl << " " << rVecIter << endl << endl;
	cout << " tvec1 " << endl << " " << tVecIter << endl << endl;

	cout << "P3P: " << endl;
	cout << " rvec1 " << endl << " " << rVecP3P << endl << endl;
	cout << " tvec1 " << endl << " " << tVecP3P << endl << endl;

	return 0;
}

PnPSolver::PnPSolver()
{
	foo();
}