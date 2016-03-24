#ifndef __CALIBRATION_H_INCLUDED__   // if x.h hasn't been included yet...
#define __CALIBRATION_H_INCLUDED__
#include <opencv2/features2d.hpp>
#include <initializer_list>
#include <iostream>
class Calibration
{
public:
	Calibration();

	static cv::Mat foo(cv::Mat cm, std::vector<std::vector<cv::Point2f> > vovIP, std::vector<std::vector<cv::Point3f> > vovWP);
	cv::Mat foo(cv::Mat cm);
	
	void setVoVImagePoints();
	void setVoVWorldPoints();

	void pushToVoVImagePoints(std::vector<cv::Point2f> IP);
	void pushToVoVWorldPoints(std::vector<cv::Point3f> WP);

private:
	cv::Mat cameraMatrix;

	std::vector<std::vector<cv::Point2f> > VoVImagePoints;
	std::vector<std::vector<cv::Point3f> > VoVWorldPoints;

	cv::Mat distortionCoeffs;
};

#endif