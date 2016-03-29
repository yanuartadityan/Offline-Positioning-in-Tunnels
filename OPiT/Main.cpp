/*
	AUTHOR: DAVID BENNEHAG

	Requires OpenCV (tested on 3.1)

		
*/
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#include "Triangulation.h"
#include "SIFTdetector.h"
#include "Calibration.h"
#include "PnPSolver.h"
#include "PointProjection.h"
#include "Converter.h"

#include <iostream>

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{

	//unsigned cts = thread::hardware_concurrency();
	//cout << cts << " concurrent threads are supported\n";

	PnPSolver solver1, solver2;
	
	//Calibration moved to its own class.
	Calibration calib;
	
	Mat distCoeffs = calib.getDistortionCoeffs();

	cout << "Starting first solver............." << endl << endl << endl;
	
	solver1.setImagePoints(vector<Point2f> { Point2d(397.210571, 145.146866), Point2d(650.494934, 129.172379), Point2d(519.567688, 131.898239), Point2d(531.834473, 267.480103), Point2d(239.835358, 207.141220),
		Point2d(834.740051, 174.580566), Point2d(211.190155, 510.402740), Point2d(437.319458, 218.244186), Point2d(845.259948, 160.41391), Point2d(559.729248, 170.678528) });
	
	// Run the PnP Solver. All matrices and stuff will be set up after this.
	solver1.foo(1);

	cout << endl << endl << endl << endl << endl << endl;
	cout << "Starting second solver............." << endl << endl << endl;
	// Take another image showing the same scene.
	//  So the 2D image points will be different of course...
	
	
	solver2.setImagePoints(vector<Point2f> { Point2d(490, 250), Point2d(668, 242), Point2d(578, 242), Point2d(582, 335), Point2d(380, 294), Point2d(793, 278), Point2d(367, 499), Point2d(521, 306), 
		Point2d(806, 262), Point2d(604, 272) });
	//  But the 3D world points will be the same.

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
	
	Mat cam1 = calib.getCameraMatrix().inv() * solver1.cameraPose34;
	Mat cam2 = calib.getCameraMatrix().inv() * solver2.cameraPose34;
	
	vector<Point2f> cam1Points = solver1.getImagePoints();
	vector<Point2f> cam2Points = solver2.getImagePoints();
	
	Mat points3D, points3Dnorm;

	triangulatePoints(
		cam1,			// 3x4 projection matrix of the first camera.
		cam2,			// 3x4 projection matrix of the second camera.
		cam1Points,		// 2xN array of feature points in the first image. In case of c++ version it can be also a vector of feature points or two - channel matrix of size 1xN or Nx1.
		cam2Points,		// 2xN array of corresponding points in the second image. In case of c++ version it can be also a vector of feature points or two - channel matrix of size 1xN or Nx1.
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
	
	//An alternative to the OpenCV triangulatePoints()
	Point3d u; u.x = cam1Points[0].x; u.y = cam1Points[0].y; u.z = 1;
	Point3d u1; u1.x = cam1Points[0].x; u1.y = cam1Points[0].y; u1.z = 1;
	Mat_<double> X_ = Triangulation::IterativeLinearLSTriangulation(u, cam1, u1, cam2);
	cout << endl <<"X_ from H&Z triangulation" << endl << X_ << endl << endl;

	
	
	SIFTdetector::foo();

	int counter = 0;

	string filename = "CADS4_OWG740_20140422_162231_005.avi";
	VideoCapture vc(filename);
	if (!vc.isOpened())
		exit(EXIT_FAILURE);

	Mat frame1, frame2;

	while (vc.read(frame2))
	{
		
		// Change brightness
		//frame2 = frame2 + Scalar(10,10,10);

		//cout << "Iteration # " << counter << endl;

		//resize(frame2, frame2, Size(frame2.cols / 2, frame2.rows / 2));

		//In the first iteration, only frame2 will contain a frame, so skip this
		if (counter == 0)
		{
			counter++;
			frame1 = frame2.clone();
			continue;
		}


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
		imshow("Disparity Map", disparity);

		Mat Q, R1, R2, P1, P2;

		stereoRectify(
			calib.getCameraMatrix(),		// First camera matrix.
			distCoeffs,						//              distcoeffs.
			calib.getCameraMatrix(),		// Second camera matrix.
			distCoeffs,						//              distcoeffs.
			Size(1280, 960),				// Size of the image used for stereo calibration.
			solver1.R,						// Rotation matrix between the coordinate systems of the two cameras.
			solver1.t,						// Translation vector between the coordinate systems of the two cameras.
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
		//cout << "First item is " << endl << _3dImage.row(0).col(0) << endl << endl;

		counter++;

		// frame1 will hold the previous frame, and in the next iteration we will read a new frame into frame2
		//    These two will thus be continuously cycled.
		frame1 = frame2.clone();

		

		if (waitKey(1) == 'k')
			break;
		
	}
	
	return 0;
}

