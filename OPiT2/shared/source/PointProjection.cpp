#include "PointProjection.h"
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
A class for projecting points from 3d to 2d

*/


void PointProjection::foo(vector<Point3f> worldPoints, Mat rVec, Mat tVec, Mat cameraMatrix, Mat distCoeffs)
{
	//MEASURE THE TIME
	int64 now, then;
	double elapsedSecondsPROJ, ticksPerSecond = cvGetTickFrequency()*1.0e6;

	then = cvGetTickCount();



	/*
	Use zero distortion, or the distcoeff calculated by calibrateCamera()?
	*/
	projectPoints(						// Projects 3D points to an image plane.
		worldPoints,					//
		rVec,							// Rotation vector
		tVec,							// Translation vector
		cameraMatrix,					//
		distCoeffs,						// Input vector of distortion coefficients. If the vector is NULL/empty, the zero distortion coefficients are assumed.
		projectionImagePoints,			// Output array of image points, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel
		projectionJacobian				// Jacobian, jacobian matrix of derivatives of image points with respect to components of the rotation vector, 
										// translation vector, focal lengths, coordinates of the principal point and the distortion coefficients.

										// Aspect ratio, optional (fx/fy)
		);


	cout << "Projected image points" << endl;
	for (int i = 0; i < projectionImagePoints.size(); i++)
	{
		cout << projectionImagePoints.at(i) << endl;
	}


	now = cvGetTickCount();
	elapsedSecondsPROJ = (double)(now - then) / ticksPerSecond;
}

std::vector<cv::Point2f> PointProjection::getProjectedImagePoints()
{
	return PointProjection::projectionImagePoints;
}

cv::Mat PointProjection::getProjectionJacobian()
{
	return PointProjection::projectionJacobian;
}
