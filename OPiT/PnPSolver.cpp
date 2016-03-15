#include "PnPSolver.h"
#include "Converter.h"

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

	solvePnP outputs the translation in the same units as we specify world points.
*/


int PnPSolver::foo()
{
	
	//MEASURE THE TIME
	int64 now, then;
	double elapsedSeconds, ticksPerSecond = cvGetTickFrequency()*1.0e6;

	//Real object points set
	//vector<Point3f> object;
	//object.push_back(Point3d(-88.0f, 88.0f, 0));
	//object.push_back(Point3d(-88.0f, -88.0f, 0));
	//object.push_back(Point3d(88.0f, -88.0f, 0));
	//object.push_back(Point3d(88.0f, 88.0f, 0));
	
	//PnP
	Mat rVecP3P, tVecP3P;

	

	if(imagePoints.size() != 4 | worldPoints.size() != 4)
	{ 
		// Keep track of time
		then = cvGetTickCount();
		
		//solvePnP(Mat(worldPoints), Mat(imagePoints), cameraMatrix, Mat(), rVecIter, tVecIter, false, CV_ITERATIVE);
		
		
		Mat inliers;
		solvePnPRansac(
			Mat(worldPoints),	// Array of world points in the world coordinate space, 3xN/Nx3 1-channel or 1xN/Nx1 3-channel, where N is the number of points.
			Mat(imagePoints),	// Array of corresponding image points, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel, where N is the number of points.
			cameraMatrix,		// Self-explanatory...
			Mat(),				// DIST COEFFS, Input vector of distortion coefficients. If null, zero distortion coefficients 
			rVecIter,			// Output rotation vector.   Together with tvec, brings points from the model coordinate system to the camera coordinate system.
			tVecIter,			// Output translation vector
			false,				// USE EXTRINSIC GUESS, if true (1), the function uses the provided rvec and tvec values as initial approximations
								//						of the rotation and translation vectors, respectively, and further optimizes them.
			100,				// ITERATIONS COUNT, number of iterations
			8,					// REPROJECTION ERROR, inlier threshold value used by the RANSAC procedure.
			0.95,				// CONFIDENCE, The probability that the algorithm produces a useful result. default 0.99;
			//100,				// INLIERS, number of inliers. If the algorithm at some stage finds more inliers than minInliersCount , it finishes.
			inliers,			// INLIERS, output vector that contains indices of inliers in worldPoints and imagePoints.
			CV_ITERATIVE);		// FLAGS, method for solving a PnP problem.

		cout << "Number of inliers: " << inliers.size() << endl;
		
		//Create the rotation matrix from the vector created above, by using the "Rodrigues"
		rMatIter.create(3, 3, DataType<double>::type);
		Rodrigues(rVecIter, rMatIter);
		//Create the translation matrix from the vector created above, by using the "Rodrigues"
		tMatIter.create(3, 3, DataType<double>::type);
		Rodrigues(tVecIter, tMatIter);

		transpose(rMatIter, rMatIter);
		invert(rMatIter, rMatIter);
		
		Mat camPos;
		cv::multiply(rMatIter, tMatIter, camPos);
		

		cout << "*******" << endl << "Camera position: " << endl << camPos << endl << "*******" << endl;

		
	
		// Calculate time>
		now = cvGetTickCount();
		elapsedSeconds = (double)(now - then) / ticksPerSecond;
		cout << "PnP (ITERATIVE) took " << elapsedSeconds << " seconds" << endl;

		//Print result
		cout << "Iterative: " << endl;
		cout << "\t R-MAT " << endl << " " << rMatIter << endl << endl;
		cout << "\t T-MAT " << endl << " " << tMatIter << endl << endl;
		
	}
	else
	{
		then = cvGetTickCount();

		// P3P requires exactly 4 points in both object and scene
		solvePnP(Mat(worldPoints), Mat(imagePoints), cameraMatrix, Mat(), rVecP3P, tVecP3P, false, CV_P3P);

		Mat rMatP3P(3, 3, DataType<double>::type);
		Rodrigues(rVecP3P, rMatP3P);

		//Mat tMatP3P(3, 3, DataType<double>::type);
		//Rodrigues(tVecP3P, tMatP3P);

		// Calculate time
		now = cvGetTickCount();
		elapsedSeconds = (double)(now - then) / ticksPerSecond;
		cout << "PnP (P3P) took " << elapsedSeconds << " seconds" << endl;
		
		cout << "P3P: " << endl;
		cout << "\t R-MAT " << endl << " " << rMatP3P << endl << endl;
		cout << "\t T-MAT " << endl << " " << tVecP3P << endl << endl;
	}
	

	return 0;
}

PnPSolver::PnPSolver()
{																//							    Camera matrix
	cameraMatrix = Mat(3, 3, CV_64FC1, Scalar::all(0));			//								___		  ___
	cameraMatrix.at<double>(0, 0) = 1432;						// Focal length X				| fx  0  cx |
	cameraMatrix.at<double>(1, 1) = 1432;						// Focal length Y				| 0  fy  cy |
	cameraMatrix.at<double>(0, 2) = 640;						// Principal point X			| 0   0   0 |
	cameraMatrix.at<double>(1, 2) = 481;						// Principal point Y			---		  ---
	cameraMatrix.at<double>(2, 2) = 1.0;						// Just a 1 cause why not	
}

PnPSolver::PnPSolver(Mat CM)
{
	if (cameraMatrix.rows != 3)
		cerr << "WRONG NR OF ROWS, MUST BE 3" << endl;
	else if (cameraMatrix.cols != 3)
		cerr << "WRONG NR OF COLUMNS, MUST BE 3" << endl;

	cameraMatrix.copyTo(PnPSolver::cameraMatrix);

	
}

// Use default image points, not recommended
void PnPSolver::setImagePoints()
{
	float	W = 4.8,
			w = 1280.0,
			H = 3.6,
			h = 960,
			f = 1432,
			F = Converter::ImageToWorldF(f, W, w);
	

	PnPSolver::imagePoints.push_back(Point2d(3.97210571e+02, 1.45146866e+02));
	PnPSolver::imagePoints.push_back(Point2d(6.50494934e+02, 1.29172379e+02));
	PnPSolver::imagePoints.push_back(Point2d(5.19567688e+02, 1.31898239e+02));
	PnPSolver::imagePoints.push_back(Point2d(5.31834473e+02, 2.67480103e+02));
	PnPSolver::imagePoints.push_back(Point2d(2.39835358e+02, 2.07141220e+02));
	PnPSolver::imagePoints.push_back(Point2d(8.34740051e+02, 1.74580566e+02));
	PnPSolver::imagePoints.push_back(Point2d(2.11190155e+02, 5.10402740e+02));


	//Converted from pixels to meters
	//PnPSolver::imagePoints.push_back(Point2d(Converter::ImageToWorldX(384.3331f, W, w), Converter::ImageToWorldY(162.23618f, H, h)));
	//PnPSolver::imagePoints.push_back(Point2d(Converter::ImageToWorldX(385.27521f, W, w), Converter::ImageToWorldY(135.21503f, H, h)));
	//PnPSolver::imagePoints.push_back(Point2d(Converter::ImageToWorldX(409.36746f, W, w), Converter::ImageToWorldY(139.30315f, H, h)));
	//PnPSolver::imagePoints.push_back(Point2d(Converter::ImageToWorldX(407.43854f, W, w), Converter::ImageToWorldY(165.64435f, H, h)));
}

void PnPSolver::setImagePoints(vector<Point2f> IP)
{
	PnPSolver::imagePoints = IP;
}

vector<cv::Point2f> PnPSolver::getImagePoints()
{
	return PnPSolver::imagePoints;
}

// Use default world points, not recommended
void PnPSolver::setWorldPoints()
{
	// 3D scene
	PnPSolver::worldPoints.push_back(Point3d(143430.318, 6394363.127, 39.797));
	PnPSolver::worldPoints.push_back(Point3d(143434.166, 6394361.662, 39.842));
	PnPSolver::worldPoints.push_back(Point3d(143432.108, 6394362.287, 39.617));
	PnPSolver::worldPoints.push_back(Point3d(143432.948, 6394364.927, 37.656));
	PnPSolver::worldPoints.push_back(Point3d(143427.658, 6394362.027, 38.376));
	PnPSolver::worldPoints.push_back(Point3d(143436.316, 6394359.472, 38.452));
	PnPSolver::worldPoints.push_back(Point3d(143462.114, 6394450.099, 38.451));
	/*
	PnPSolver::worldPoints.push_back(Point3d(143469.613, 6394456.418, 38.800));
	PnPSolver::worldPoints.push_back(Point3d(143468.953, 6394441.302, 37.281));
	PnPSolver::worldPoints.push_back(Point3d(143466.173, 6394449.469, 38.671));
	PnPSolver::worldPoints.push_back(Point3d(143467.283, 6394451.589, 38.711));
	PnPSolver::worldPoints.push_back(Point3d(143468.983, 6394441.362, 36.431));
	PnPSolver::worldPoints.push_back(Point3d(143427.048, 6394361.520, 33.577));
	*/


	/*
	PnPSolver::worldPoints.push_back(Point3d(143436.316, 6394359.472, 38.452));
	PnPSolver::worldPoints.push_back(Point3d(143427.658, 6394362.027, 38.376));
	PnPSolver::worldPoints.push_back(Point3d(143468.983, 6394441.362, 36.431));
	PnPSolver::worldPoints.push_back(Point3d(143468.953, 6394441.302, 37.281));
	PnPSolver::worldPoints.push_back(Point3d(143462.114, 6394450.099, 38.451));
	PnPSolver::worldPoints.push_back(Point3d(143467.283, 6394451.589, 38.711));
	PnPSolver::worldPoints.push_back(Point3d(143466.173, 6394449.469, 38.671));
	PnPSolver::worldPoints.push_back(Point3d(143469.613, 6394456.418, 38.800));
	*/
}

void PnPSolver::setWorldPoints(vector<Point3f> WP)
{
	PnPSolver::worldPoints = WP;
}

vector<cv::Point3f> PnPSolver::getWorldPoints()
{
	return PnPSolver::worldPoints;
}


Mat PnPSolver::getRotationVector()
{
	return PnPSolver::rVecIter;
}
Mat PnPSolver::getRotationMatrix()
{
	return PnPSolver::rMatIter;
}
Mat PnPSolver::getTranslationVector()
{
	return PnPSolver::tVecIter;
}
Mat PnPSolver::getTranslationMatrix()
{
	return PnPSolver::tMatIter;
}
Mat PnPSolver::getCameraMatrix()
{
	return PnPSolver::cameraMatrix;
}