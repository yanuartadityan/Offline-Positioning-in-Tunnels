#include <opencv2/opencv.hpp>

#include "Triangulation.h"
#include "Calibration.h"

using namespace std;
using namespace cv;


// Should use undistortPoints() and then pass into correctMatches() to refine the coordinates as they were handpicked?
//   Discussed here: http://www.answers.opencv.org/question/3275/is-triangulatepoints-outputing-rubish/

// Is triangulatePoints() the next step?   (Points need to be "undistorted" first. Either remove distortion before picking points, or undistort the points)

// The triangulation in OpenCV is apparently crap, should use equations from H&Z instead,
//    as described here: http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/#more-1023
// Or use the example here: http://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints

/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997

Mat LinearLSTriangulation(
	Point3d u,       //homogenous image point (u,v,1)
	Matx34d P,       //camera 1 matrix
	Point3d u1,      //homogenous image point in 2nd camera
	Matx34d P1       //camera 2 matrix
)*/
cv::Mat Triangulation::LinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1)
{
	//build matrix A for homogenous equation system Ax = 0
	//assume X = (x,y,z,1), for Linear-LS method
	//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	Matx43d A(
		u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
		);


	Mat temp; temp = -(u.x*P(2, 3) - P(0, 3));
	Matx41d B;
	B = (temp,
		-(u.y*P(2, 3) - P(1, 3)),
		-(u1.x*P1(2, 3) - P1(0, 3)),
		-(u1.y*P1(2, 3) - P1(1, 3)));


	Mat X;
	solve(A, B, X, DECOMP_SVD);

	return X;
}



/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997

cv::Mat_<double> IterativeLinearLSTriangulation(
	cv::Point3d u,		    //homogenous image point (u,v,1)
	cv::Matx34d P,          //camera 1 matrix
	cv::Point3d u1,         //homogenous image point in 2nd camera
	cv::Matx34d P1          //camera 2 matrix
)*/
cv::Mat_<double> Triangulation::IterativeLinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1)
{
	double wi = 1, wi1 = 1;
	Mat_<double> X(4, 1);
	for (int i = 0; i<10; i++) { //Hartley suggests 10 iterations at most
		Mat_<double> X_ = LinearLSTriangulation(u, P, u1, P1);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;

		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);

		//breaking point
		//if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

		wi = p2x;
		wi1 = p2x1;

		//reweight equations and solve
		Matx43d A((u.x*P(2, 0) - P(0, 0)) / wi, (u.x*P(2, 1) - P(0, 1)) / wi, (u.x*P(2, 2) - P(0, 2)) / wi,
			(u.y*P(2, 0) - P(1, 0)) / wi, (u.y*P(2, 1) - P(1, 1)) / wi, (u.y*P(2, 2) - P(1, 2)) / wi,
			(u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
			(u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1
			);
		Mat_<double> B = (Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)) / wi,
			-(u.y*P(2, 3) - P(1, 3)) / wi,
			-(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
			-(u1.y*P1(2, 3) - P1(1, 3)) / wi1
			);

		solve(A, B, X_, DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
	}
	return X;
}




Mat Triangulation::ocvTriangulation(Mat cPose1, Mat cPose2, vector<Point2f> imgPoints1, vector<Point2f> imgPoints2)
{
	Calibration calib;

	Mat cam1 = calib.getCameraMatrix().inv() * cPose1;
	Mat cam2 = calib.getCameraMatrix().inv() * cPose2;

	Mat points3D, points3Dnorm;

	triangulatePoints(
		cam1,			// 3x4 projection matrix of the first camera.
		cam2,			// 3x4 projection matrix of the second camera.
		imgPoints1,		// 2xN array of feature points in the first image. In case of c++ version it can be also a vector of feature points or two - channel matrix of size 1xN or Nx1.
		imgPoints2,		// 2xN array of corresponding points in the second image. In case of c++ version it can be also a vector of feature points or two - channel matrix of size 1xN or Nx1.
		points3D		// 4xN array of reconstructed points in homogeneous coordinates.
		);

	points3D.copyTo(points3Dnorm);

	for (int i = 0; i < points3D.cols; i++)
	{
		points3Dnorm.at<float>(0, i) = points3D.at<float>(0, i) / points3D.at<float>(3, i);
		points3Dnorm.at<float>(1, i) = points3D.at<float>(1, i) / points3D.at<float>(3, i);
		points3Dnorm.at<float>(2, i) = points3D.at<float>(2, i) / points3D.at<float>(3, i);

		cout << "************************************" << endl
			<< "X = " << points3Dnorm.at<float>(0, i) << endl
			<< "Y = " << points3Dnorm.at<float>(1, i) << endl
			<< "Z = " << points3Dnorm.at<float>(2, i) << endl
			<< "************************************" << endl;

	}


	return points3Dnorm;
}


