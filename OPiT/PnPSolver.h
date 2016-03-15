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
	std::vector<cv::Point3f> getWorldPoints();

	void setImagePoints();
	void setImagePoints(std::vector<cv::Point2f> IP);
	std::vector<cv::Point2f> getImagePoints();

	cv::Mat getCameraMatrix();

	cv::Mat getRotationVector();
	cv::Mat getTranslationVector();

	cv::Mat getRotationMatrix();
	cv::Mat getTranslationMatrix();

	int foo();

private:
	cv::Mat cameraMatrix;
	std::vector<cv::Point2f> imagePoints;
	std::vector<cv::Point3f> worldPoints;
	cv::Mat rVecIter, tVecIter;
	cv::Mat rMatIter, tMatIter;
	
};

#endif