/*
	AUTHOR: DAVID BENNEHAG

	Requires OpenCV (tested on 3.1)

		
*/
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "PnPSolver.h"

#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	PnPSolver solver;
	solver.setImagePoints();
	solver.setWorldPoints();

	solver.foo();
	vector<cv::Point2f> imagePoints;
	Mat jacobian;

	projectPoints(						// Projects 3D points to an image plane.
		solver.getWorldPoints(),		//
		solver.getRotationVector(),		//
		solver.getTranslationVector(),	//
		solver.getCameraMatrix(),		//
		Mat(),							// Input vector of distortion coefficients. If the vector is NULL/empty, the zero distortion coefficients are assumed.
		imagePoints,					// Output array of image points, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel
		jacobian						// Jacobian, jacobian matrix of derivatives of image points with respect to components of the rotation vector, 
										// translation vector, focal lengths, coordinates of the principal point and the distortion coefficients.
										
										// Aspect ratio, optional (fx/fy)
		);

	cout << "Size of imagePoints: " << imagePoints.size() << endl;

	cout << solver.getImagePoints() << endl;

	for (Point2f x : imagePoints)
		cout << "point2f = " << x << endl;


	// Transpose the rotation matrix 
	Mat rMatTra = solver.getRotationMatrix().t();
	// Invert the translation vector
	Mat tVecInv = -rMatTra * solver.getTranslationVector();

	Mat camPose(4, 4, solver.getRotationMatrix().type());
	
	//Copies the rotation matrix into the camera pose
	camPose(Range(0, 3), Range(0, 3)) = rMatTra * 1;
	
	//Copies tvec into the camera pose
	camPose( Range(0,3), Range(3,4) ) = tVecInv * 1;
	
	// Fill the last row of the camera pose matrix with [0, 0, 0, 1]
	double *p = camPose.ptr<double>(3);
	p[0] = p[1] = p[2] = 0; p[3] = 1;

	cout << "Camera pose: " << endl << camPose << endl;
}