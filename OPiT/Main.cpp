/*
	AUTHOR: DAVID BENNEHAG

	Requires OpenCV (tested on 3.1)


*/
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
#include "Triangulation.h"
#include "FeatureDetection.h"
#include "Calibration.h"
#include "PnPSolver.h"
#include "PointProjection.h"
#include "Converter.h"
#include "VisualOdometry.h"
#include "PCLTest.h"

#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	//Calibration moved to its own class.
    Calibration calib;
	PnPSolver solver1, solver2;
	VO vodometry;
    FeatureDetection fdetect;

    Mat distCoeffs = calib.getDistortionCoeffs();

#ifndef KITTI_DATASET
    char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/volvo/out0/";
#else
    char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/kitti-odometry/dataset/sequences/00/image_0/";
#endif

    // CHECKING THE IMAGE QUALITY PER DIFFERENT FEATURE DETECTION
    // fdetect.computeKeypointsAndDraw(pathname);

    // VISUAL ODOMETRY
    //vodometry.initParameter();
    //vodometry.setImagePath(pathname);
    //vodometry.visualodometry();

    // MANUAL CORRESPONDENCES PNP SOLVER
	cout << "Starting first solver............." << endl << endl << endl;

	vector<Point2d> imageOne;
	imageOne.push_back(Point2d(397.210571, 145.146866));	imageOne.push_back(Point2d(650.494934, 129.172379));
	imageOne.push_back(Point2d(519.567688, 131.898239));	imageOne.push_back(Point2d(531.834473, 267.480103));
	imageOne.push_back(Point2d(239.835358, 207.141220));	imageOne.push_back(Point2d(834.740051, 174.580566));
	imageOne.push_back(Point2d(211.190155, 510.402740));	imageOne.push_back(Point2d(437.319458, 218.244186));
	imageOne.push_back(Point2d(845.259948, 160.413910));	imageOne.push_back(Point2d(559.729248, 170.678528));

	solver1.setImagePoints(imageOne);
	solver1.foo(1);

	cout << endl << endl << endl << endl << endl << endl;
	cout << "Starting second solver............." << endl << endl << endl;

	// Take another image showing the same scene.
	//  So the 2D image points will be different of course...

	vector<Point2d> imageTwo;
	imageTwo.push_back(Point2d(490, 250));	imageTwo.push_back(Point2d(668, 242));
	imageTwo.push_back(Point2d(578, 242));	imageTwo.push_back(Point2d(582, 335));
	imageTwo.push_back(Point2d(380, 294));	imageTwo.push_back(Point2d(793, 278));
	imageTwo.push_back(Point2d(367, 499));	imageTwo.push_back(Point2d(521, 306));
	imageTwo.push_back(Point2d(806, 262));	imageTwo.push_back(Point2d(604, 272));

	solver2.setImagePoints(imageTwo);
	solver2.foo(1);

	cout << endl << endl << endl << endl << endl << endl;
	cout << endl << endl << "camera 1 position: " << endl << solver1.getCameraPosition() << endl << "camera 2 position: " << endl << solver2.getCameraPosition() << endl;

    //
    //// IDK WHAT IS THIS
	//PCLTest::foo();
	//FeatureDetection::foo();
	//
	//int counter = 0;
	//
	//string filename = "CADS4_OWG740_20140422_162231_005.avi";
	//VideoCapture vc(filename);
	//if (!vc.isOpened())
	//	exit(EXIT_FAILURE);
	//
	//Mat frame1, frame2;

	// Load the point cloud only once, as it is very slow
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("cloud.pcd", *cloud);

	/*
		Projection algorithm from: http://stackoverflow.com/questions/13957150/opencv-computing-camera-position-rotation

		Multiply camera pose (T) with the homogeneous coordinates for each point, with Z-values from 0-n, to get the points of the ray.
	*/
	Mat T = solver1.getCameraPose().clone();
	Mat p, p_, p__, p___;
	double newX, newY, newZ;

	vector<double> bestPoint{ 0, 0, 0, 1000 };

    // reproject 2D to 3D
    Mat K = calib.getCameraMatrix();
    for (int pidx=0; pidx<10; pidx++)
    {
        int limit = 0;
        int counter = 0;
        int counterIdx = 0;

        // get the current pixels
        cout << imageOne[pidx] << endl;

    	for (double i = 20; i < 40; i+=0.4)
    	{
            // build a line originating from camera center (0,0,0) to the pixels
            p___ = (Mat_<double>(3,1) << i*(imageOne[pidx].x), i*(imageOne[pidx].y), i);

            p__ = K.inv() * p___;

            p_ = (Mat_<double>(4,1) << p__.at<double>(0,0), p__.at<double>(1,0), p__.at<double>(2,0), 1);

            p = T * p_;

    		newX = p.at<double>(0, 0); newY = p.at<double>(1, 0); newZ = p.at<double>(2, 0);

    		vector<double> newPoint = PCLTest::test(newX, newY, newZ, cloud);

    		limit++;

//    		if (newPoint[3] < bestPoint[3])
            if (newPoint[3] < 0.01f)
    		{
    			bestPoint = newPoint;
    			limit = 0;
                counterIdx = counter;
                break;
    		}

    		// cout << "i = \t"    << i    << endl
    		// 	 << "newX = \t" << newX << endl
    		// 	 << "newY = \t" << newY << endl
    		// 	 << "newZ = \t" << newZ << endl
    		// 	 << endl << endl;

           counter++;
    	}

        cout.setf(ios::fixed);
    	cout << "*****************************\n";
    	cout << "The best point found after "<< counter << " iteration in " << counterIdx << "th:\n"
    		<< "X = \t" << std::setprecision(5) << bestPoint[0] << endl
    		<< "Y = \t" << std::setprecision(5) << bestPoint[1] << endl
    		<< "Z = \t" << std::setprecision(5) << bestPoint[2] << endl
    		<< "DIST = \t" << bestPoint[3] << endl;
    	cout << "*****************************\n";

        // reset
        bestPoint = {0, 0, 0, 1000};
    }

	//cout	<< "increments in X: \t " << (newX - prevX) << endl
	//		<< "increments in Y: \t " << (newY - prevY) << endl
	//		<< "increments in Z: \t " << (newZ - prevZ) << endl
	//		<< endl;

	//Mat p1_ = T * p1;
	//Mat p2_ = T * p2;

	//cout << "p1_ = " << endl << p1_ << endl << endl
	//	<< "p2_ = " << endl << p2_ << endl << endl;
	/*
	// Skeleton code for iterating through the image sequence
	while (vc.read(frame2))
	{

		// Change brightness
		//frame2 = frame2 + Scalar(10,10,10);


		//resize(frame2, frame2, Size(frame2.cols / 2, frame2.rows / 2));

		//In the first iteration, only frame2 will contain a frame, so skip this
		if (counter == 0)
		{
			counter++;
			frame1 = frame2.clone();
			continue;
		}

		counter++;

		// frame1 will hold the previous frame, and in the next iteration we will read a new frame into frame2
		//    These two will thus be continuously cycled.
		frame1 = frame2.clone();

		if (waitKey(5000) == 'k')
			break;
	}
	*/



	return 0;
}
