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
#include "PCLTest.h"

#include <iostream>

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
	//Calibration moved to its own class.
	Calibration calib;
	Mat distCoeffs = calib.getDistortionCoeffs();


	PnPSolver solver1, solver2;

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
	imageTwo.push_back(Point2d(490, 250));
	imageTwo.push_back(Point2d(668, 242));
	imageTwo.push_back(Point2d(578, 242));
	imageTwo.push_back(Point2d(582, 335));
	imageTwo.push_back(Point2d(380, 294));
	imageTwo.push_back(Point2d(793, 278));
	imageTwo.push_back(Point2d(367, 499));
	imageTwo.push_back(Point2d(521, 306));
	imageTwo.push_back(Point2d(806, 262));
	imageTwo.push_back(Point2d(604, 272));


	solver2.setImagePoints(imageTwo);
	solver2.foo(1);

	cout << endl << endl << endl << endl << endl << endl;
	cout << endl << endl << "camera 1 position: " << endl << solver1.getCameraPosition() << endl << "camera 2 position: " << endl << solver2.getCameraPosition() << endl;

	PCLTest::foo();


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






		counter++;

		// frame1 will hold the previous frame, and in the next iteration we will read a new frame into frame2
		//    These two will thus be continuously cycled.
		frame1 = frame2.clone();



		if (waitKey(5000) == 'k')
			break;

	}

	return 0;
}
