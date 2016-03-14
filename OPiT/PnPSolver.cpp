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
	//Camera matrix
	Mat cam_matrix = Mat(3, 3, CV_64FC1, Scalar::all(0));
	cam_matrix.at<double>(0, 0) = 1432;						// Focal length X
	cam_matrix.at<double>(1, 1) = 1432;						// Focal length Y
	cam_matrix.at<double>(0, 2) = 640;						// Principal point X
	cam_matrix.at<double>(1, 2) = 481;						// Principal point Y
	cam_matrix.at<double>(2, 2) = 1.0;						// Just a 1 cause why not

	now = cvGetTickCount();
	elapsedSeconds = (double)(now - then) / ticksPerSecond;

	cout << "Initializtion took " << elapsedSeconds << " seconds" << endl;

	//PnP
	Mat rvec1i, rvec2i, tvec1i, tvec2i;
	Mat rvec1p, rvec2p, tvec1p, tvec2p;


	then = cvGetTickCount();

	solvePnP(Mat(object), Mat(points1), cam_matrix, Mat(), rvec1i, tvec1i, false, CV_ITERATIVE);
	//solvePnP(Mat(object), Mat(points2), cam_matrix, Mat(), rvec2i, tvec2i, false, CV_ITERATIVE);

	now = cvGetTickCount();
	elapsedSeconds = (double)(now - then) / ticksPerSecond;

	cout << "PnP (ITERATIVE) took " << elapsedSeconds << " seconds" << endl;

	then = cvGetTickCount();

	solvePnP(Mat(object), Mat(points1), cam_matrix, Mat(), rvec1p, tvec1p, false, CV_P3P);
	//solvePnP(Mat(object), Mat(points2), cam_matrix, Mat(), rvec2p, tvec2p, false, CV_P3P);

	now = cvGetTickCount();
	elapsedSeconds = (double)(now - then) / ticksPerSecond;

	cout << "PnP (P3P) took " << elapsedSeconds << " seconds" << endl;


	//Print result
	cout << "Iterative: " << endl;
	cout << " rvec1 " << endl << " " << rvec1i << endl << endl;
	cout << " rvec2 " << endl << " " << rvec2i << endl << endl;
	cout << " tvec1 " << endl << " " << tvec1i << endl << endl;
	cout << " tvec1 " << endl << " " << tvec2i << endl << endl;

	cout << "P3P: " << endl;
	cout << " rvec1 " << endl << " " << rvec1p << endl << endl;
	cout << " rvec2 " << endl << " " << rvec2p << endl << endl;
	cout << " tvec1 " << endl << " " << tvec1p << endl << endl;
	cout << " tvec1 " << endl << " " << tvec2p << endl << endl;

	return 0;
}

PnPSolver::PnPSolver()
{
	foo();
}