#ifndef __PnPSolver_H_INCLUDED__   // if x.h hasn't been included yet...
#define __PnPSolver_H_INCLUDED__
#include <opencv2/features2d.hpp>
#include <initializer_list>
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

	void setVoVImagePoints();

	cv::Mat getCameraMatrix();
	cv::Mat getCameraPose();
	cv::Mat getCameraPosition();

	cv::Mat getRotationVector();
	cv::Mat getTranslationVector();

	cv::Mat getRotationMatrix();
	cv::Mat getTranslationMatrix();

	

	std::vector<cv::Point2f> getProjectedImagePoints();
	cv::Mat getProjectionJacobian();
	
	int foo(int verbal);

private:
	cv::Mat cameraMatrix;
	
	std::vector<cv::Point2f> imagePoints, imagePoints2;
	std::vector<cv::Point3f> worldPoints;

	std::vector<std::vector<cv::Point2f> > VoVImagePoints;

	cv::Mat rVecIter, tVecIter;
	cv::Mat rMatIter, tMatIter;
	cv::Mat cameraPose, cameraPosition;
	cv::Mat distCoeffs;

	std::vector<cv::Point2f> projectedImagePoints;
	cv::Mat projectionJacobian;
};

#endif