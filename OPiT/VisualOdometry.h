#ifndef _VISUAL_ODOMETRY_HEADER
#define _VISUAL_ODOMETRY_HEADER

#include <opencv2/features2d.hpp>
#include <initializer_list>
#include <iostream>

enum
{
    VO_METHOD_FAST = 0,
    VO_METHOD_SIFT = 1,
    VO_METHOD_SURF = 2
};

class VO
{
public:
    VO();

    void initParameter();
    void setImagePath(char *pathname);
    void visualodometry();
    void featureDetection(cv::Mat img, std::vector<cv::Point2f>& points, int vo_method);
    void featureTracking();
    
private:
    cv::Mat cameraMatrix;

	cv::Mat rVec, tVec;
	cv::Mat rMat, tMat;
	cv::Mat cameraPose, cameraPosition;

    char imagePath[100];
    

    
    // param
    int iterationsCount;
    float reprojectionError;
    double confidence;
    int method;
    
    // fast param
    int fast_threshold;
    bool nonMaxSuppression;
    
    // surf param
    int min_hessian;
    
    // sift param
};

#endif
