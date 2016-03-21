/*
	AUTHOR: DAVID BENNEHAG

	Requires OpenCV (tested on 3.1)

		
*/
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "PnPSolver.h"
#include "PointProjection.h"
#include "Converter.h"

#include <iostream>

using namespace std;
using namespace cv;

Mat LinearLSTriangulation(Point3d u, Matx34d P,	Point3d u1,	Matx34d P1);

int main(int argc, char** argv)
{
	PnPSolver solver1, solver2;

	cout << "Starting first solver............." << endl << endl << endl;
	vector<Point2f> imagepoints;
	imagepoints.push_back(Point2d(397.210571, 145.146866)); imagepoints.push_back(Point2d(650.494934, 129.172379)); imagepoints.push_back(Point2d(519.567688, 131.898239)); 
	imagepoints.push_back(Point2d(531.834473, 267.480103)); imagepoints.push_back(Point2d(239.835358, 207.141220)); imagepoints.push_back(Point2d(834.740051, 174.580566)); 
	imagepoints.push_back(Point2d(211.190155, 510.402740)); imagepoints.push_back(Point2d(437.319458, 218.244186)); imagepoints.push_back(Point2d(845.259948, 160.41391)); 
	imagepoints.push_back(Point2d(559.729248, 170.678528));
	
	solver1.setVoVImagePoints();
	solver1.setImagePoints(imagepoints);
	solver1.setWorldPoints();
	// Run the PnP Solver. All matrices and stuff will be set up after this.
	solver1.foo(1);

	cout << endl << endl << endl << endl << endl << endl;
	cout << "Starting second solver............." << endl << endl << endl;
	// Take another image showing the same scene.
	//  So the 2D image points will be different of course...
	vector<Point2f> imagepoints2;
	imagepoints2.push_back(Point2d(490, 250)); imagepoints2.push_back(Point2d(668, 242)); imagepoints2.push_back(Point2d(578, 242)); imagepoints2.push_back(Point2d(582, 335));
	imagepoints2.push_back(Point2d(380, 294)); imagepoints2.push_back(Point2d(793, 278)); imagepoints2.push_back(Point2d(367, 499)); imagepoints2.push_back(Point2d(521, 306));
	imagepoints2.push_back(Point2d(806, 262)); imagepoints2.push_back(Point2d(604, 272));
	solver2.setVoVImagePoints();
	solver2.setImagePoints(imagepoints2);
	//  But the 3D world points will be the same.
	solver2.setWorldPoints();

	solver2.foo(1);


	cout << endl << endl << endl << endl << endl << endl;
	cout << endl << endl << "camera 1 position: " << endl << solver1.getCameraPosition() << endl << "camera 2 position: " << endl << solver2.getCameraPosition() << endl;
	
	// Should use undistortPoints() and then pass into correctMatches() to refine the coordinates as they were handpicked?
	//   Discussed here: http://www.answers.opencv.org/question/3275/is-triangulatepoints-outputing-rubish/

	// Is triangulatePoints() the next step?   (Points need to be "undistorted" first. Either remove distortion before picking points, or undistort the points)

	// The triangulation in OpenCV is apparently crap, should use equations from H&Z instead,
	//    as described here: http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/#more-1023
	// Or use the example here: http://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints


	/*
	triangulatePoints: Reconstructs points by triangulation.
	*/
	
	Mat cam1 = solver1.getCameraMatrix() * solver1.cameraPose34;
	Mat cam2 = solver2.getCameraMatrix() * solver2.cameraPose34;
	
	vector<Point2f> cam1Points = solver1.getImagePoints();
	vector<Point2f> cam2Points = solver2.getImagePoints();
	
	Mat points3D;

	triangulatePoints(
		cam1,			// 3x4 projection matrix of the first camera.
		cam2,			// 3x4 projection matrix of the second camera.
		cam1Points,		// 2xN array of feature points in the first image. In case of c++ version it can be also a vector of feature points or two - channel matrix of size 1xN or Nx1.
		cam2Points,		// 2xN array of corresponding points in the second image. In case of c++ version it can be also a vector of feature points or two - channel matrix of size 1xN or Nx1.
		points3D		// 4xN array of reconstructed points in homogeneous coordinates.
		);

	cout << endl << "****************************" << endl << "points3D = " << endl << points3D << endl;
	/*
	//An alternative to the OpenCV triangulatePoints()
	Point3d u; u.x = cam1Points[0].x; u.y = cam1Points[0].y; u.z = 1;
	Point3d u1; u1.x = cam1Points[0].x; u1.y = cam1Points[0].y; u1.z = 1;
	Mat_<double> X_ = LinearLSTriangulation(u, cam1, u1, cam2);
	cout << X_ << endl;

	*/
	cout << endl << endl << endl << endl << endl << endl;
	cout << "Fundamental Mat = " << endl << findFundamentalMat(solver1.getImagePoints(), solver2.getImagePoints(), CV_FM_RANSAC) << endl << endl;
	



	return 0;
}

/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
Mat LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
						  Matx34d P,       //camera 1 matrix
						  Point3d u1,      //homogenous image point in 2nd camera
						  Matx34d P1       //camera 2 matrix
	)
{
	cout << "Creating A" << endl;
	//build matrix A for homogenous equation system Ax = 0
	//assume X = (x,y,z,1), for Linear-LS method
	//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	Matx43d A(
		u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
		);
	cout << "Creating B" << endl;
	
	Mat temp; temp = -(u.x*P(2, 3) - P(0, 3));
	Matx41d B;
		B = (temp,
		-(u.y*P(2, 3) - P(1, 3)),
		-(u1.x*P1(2, 3) - P1(0, 3)),
		-(u1.y*P1(2, 3) - P1(1, 3)));

	cout << "Creating X" << endl;
	Mat X;
	solve(A, B, X, DECOMP_SVD);

	return X;
}