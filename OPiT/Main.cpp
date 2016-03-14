/*
	AUTHOR: DAVID BENNEHAG

	Requires OpenCV (tested on 3.1)

		
*/

#include "PnPSolver.h"

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{


	PnPSolver solver;
	solver.setImagePoints();
	solver.setWorldPoints();

	solver.foo();
}