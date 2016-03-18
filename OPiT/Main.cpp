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
	PnPSolver solver, solver2;

	cout << "Starting first solver............." << endl << endl << endl;
	
	vector<Point2f> imagepoints;
	imagepoints.push_back(Point2d(397.210571, 145.146866));
	imagepoints.push_back(Point2d(650.494934, 129.172379));
	imagepoints.push_back(Point2d(519.567688, 131.898239));
	imagepoints.push_back(Point2d(531.834473, 267.480103));
	imagepoints.push_back(Point2d(239.835358, 207.141220));
	imagepoints.push_back(Point2d(834.740051, 174.580566));
	imagepoints.push_back(Point2d(211.190155, 510.402740));
	imagepoints.push_back(Point2d(437.319458, 218.244186));
	imagepoints.push_back(Point2d(845.259948, 160.41391));
	imagepoints.push_back(Point2d(559.729248, 170.678528));
	
	solver.setVoVImagePoints();
	solver.setImagePoints(imagepoints);
	solver.setWorldPoints();
	
	// Run the PnP Solver. All matrices and stuff will be set up after this.
	solver.foo(1);


	cout << "Starting second solver............." << endl << endl << endl;


	// Take another image showing the same scene.
	//  So the 2D image points will be different of course...
	vector<Point2f> imagepoints2;
	imagepoints2.push_back(Point2d(490, 250));
	imagepoints2.push_back(Point2d(668, 242));
	imagepoints2.push_back(Point2d(578, 242));
	imagepoints2.push_back(Point2d(582, 335));
	imagepoints2.push_back(Point2d(380, 294));
	imagepoints2.push_back(Point2d(793, 278));
	imagepoints2.push_back(Point2d(368, 503));
	imagepoints2.push_back(Point2d(521, 306));
	imagepoints2.push_back(Point2d(806, 262));
	imagepoints2.push_back(Point2d(604, 272));
	
	solver2.setVoVImagePoints();
	solver2.setImagePoints(imagepoints2);
	//  But the 3D world points will be the same.
	solver2.setWorldPoints();

	solver2.foo(2);

	cout << endl << endl << "camera 1 position: " << endl << solver.getCameraPosition() << endl << "camera 2 position: " << endl << solver2.getCameraPosition() << endl;
	
	// Should use undistortPoints() and then pass into correctMatches() to refine the coordinates as they were handpicked?
	//   Discussed here: http://www.answers.opencv.org/question/3275/is-triangulatepoints-outputing-rubish/

	// Is triangulatePoints() the next step?   (Points need to be "undistorted" first. Either remove distortion before picking points, or undistort the points)

	// The triangulation in OpenCV is apparently crap, should use equations from H&Z instead,
	//    as described here: http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/#more-1023
	// Or use the example here: http://stackoverflow.com/questions/16295551/how-to-correctly-use-cvtriangulatepoints


	return 0;
}