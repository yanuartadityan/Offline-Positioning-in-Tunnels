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