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

	// Run the PnP Solver. All matrices and stuff will be set up after this.
	solver.foo();


	cout << "*****" << endl << "The projection Jacobian: " << endl << solver.getProjectionJacobian() << endl << "*****" << endl;

	cout << endl << "Size of imagePoints: " << solver.getProjectedImagePoints().size() << endl;

	cout << solver.getImagePoints() << endl;

	for (Point2f x : solver.getProjectedImagePoints())
		cout << "point2f = " << x << endl;

	cout << "*****" << endl;
	



	cout << "Camera pose: " << endl << solver.getCameraPose() << endl;
}