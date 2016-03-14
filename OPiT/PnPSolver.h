#ifndef __PnPSolver_H_INCLUDED__   // if x.h hasn't been included yet...
#define __PnPSolver_H_INCLUDED__
#include <opencv2/features2d.hpp>
#include <iostream>
class PnPSolver
{
public:
	PnPSolver();
	PnPSolver(cv::Mat CM);
	
	void setWorldPoints();
	void setWorldPoints(std::vector<cv::Point3f> WP);

	void setImagePoints();
	void setImagePoints(std::vector<cv::Point2f> IP);

	int foo();

private:
	cv::Mat cameraMatrix;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> worldPoints;
};

#endif