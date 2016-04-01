#include "Reprojection.h"
#include "Calibration.h"

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace std;
using namespace cv;

Mat Reprojection::foo(Mat frame1, Mat frame2, Mat rMat1, Mat rMat2, cv::Mat tVec1, cv::Mat tVec2)
{
	Calibration calib;
	Mat distCoeffs = calib.getDistortionCoeffs();

	Mat frame1g, frame2g;
	// For StereoBM, images have to be CV_8UC1, original was CV_8UC3
	cvtColor(frame2, frame2g, CV_BGRA2GRAY);
	cvtColor(frame1, frame1g, CV_BGRA2GRAY);

	//printf("Matrix: %s \n", type2str(frame1g.type()).c_str());
	//printf("Matrix: %s \n", type2str(frame2g.type()).c_str());

	Mat disparity;

	Ptr<StereoBM> sbm = StereoBM::create(
		5 * 16,		// ndisparities, the disparity search range. For each pixel, the algorithm will find the best disparity from 0 (default minimum disparity) to ndisparities. 
					//	The search range can then be shifted by changing the minimum disparity.

		5			// SADWindowSize, the linear size of the blocks compared by the algorithm. The size should be odd (as the block is centered at the current pixel). 
					//	Larger block size implies smoother, though less accurate disparity map. Smaller block size gives more detailed disparity map, 
					//		but there is higher chance for algorithm to find a wrong correspondence.

		);

	sbm->compute(frame1g, frame2g, disparity);

	//imshow("frame1", frame1);
	//imshow("frame2", frame2);
	//imshow("Disparity Map", disparity);

	Mat Q = Mat(4, 4, CV_64F),
	R1 = Mat(3, 3, CV_64F),
	R2 = Mat(3, 3, CV_64F),
	P1 = Mat(3, 4, CV_64F),
	P2 = Mat(3, 4, CV_64F);
	/*
	The rotation and translation we give stereoRectify() has to be the rotation and translation between the two cameras,
	not between the two cameras and the world separately.

	From: http://stackoverflow.com/questions/5055864/finding-rotation-matrices-between-two-cameras-for-stereorectify

	For rotation between camera 1 and camera 2, use inverse extrinsic matrix of camera 1, multiplied by the extrinsic matrix of camera 2
	*/


	Mat rotation = rMat1.inv() * rMat2;
	cout << "Rotation = " << endl << rotation << endl;

	Mat translation = tVec1 - tVec2;
	cout << "Translation = " << endl << translation << endl << endl;

	stereoRectify(
		calib.getCameraMatrix(),		// First camera matrix.
		distCoeffs,						//              distcoeffs.
		calib.getCameraMatrix(),		// Second camera matrix.
		distCoeffs,						//              distcoeffs.
		Size(1280, 960),				// Size of the image used for stereo calibration.
		rotation,						// Rotation matrix between the coordinate systems of the two cameras.
		translation,					// Translation vector between the coordinate systems of the two cameras.
		R1,								// Output
		R2,								// Output
		P1,								// Output
		P2,								// Output
		Q								// Output 4x4 disparity-to-depth mapping matrix, to be used in reprojectImageTo3D().
										// Optional flags, should this be set?
		);

	Mat _3dImage;
	reprojectImageTo3D(
		disparity,		// Input disparity image.
		_3dImage,		// Output image of the same size as disp. Each element of _3dImage(x,y) contains 3D coordinates of the point (x,y) computed from the disparity map.
		Q,				// 4x4 perspective transformation matrix that can be obtained by stereoRectify().
		true			// handleMissingValues indicates whether the function should handle missing values (i.e. points where the disparity was not computed).
		);

	//cout << "size of _3dImage is " << _3dImage.size() << endl;
	cout << "An item from _3dImage = " << endl << _3dImage.row(490).col(250) << endl << endl;

	return _3dImage;
}