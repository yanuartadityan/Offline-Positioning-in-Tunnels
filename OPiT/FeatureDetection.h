#ifndef FEATUREDETECTION_H
#define FEATUREDETECTION_H

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <initializer_list>
#include <iostream>

// TODO: future mod on the class naming
class FeatureDetection
{
public:
    // constructor
    FeatureDetection() : sift_matching_ratio(0.8f)
    {
        // fast
        fast_threshold = 20;
        nonMaxSuppression = true;

        // surf
        min_hessian = 200;
        octave_layer = 3;
        contrast_threshold = 0.04;
        edge_threshold = 10;
        sigma = 1.6;

        // sift
        sift_matching_ratio = 0.8;

        //detector
        fastdetect_ = cv::FastFeatureDetector::create(fast_threshold, nonMaxSuppression, cv::FastFeatureDetector::TYPE_9_16);
        siftdetect_ = cv::xfeatures2d::SIFT::create(0, octave_layer, contrast_threshold, edge_threshold, sigma);
        surfdetect_ = cv::xfeatures2d::SURF::create(min_hessian);

        //extractor
        siftextract_ = cv::xfeatures2d::SIFT::create(0, octave_layer, contrast_threshold, edge_threshold, sigma);
        surfextract_ = cv::xfeatures2d::SURF::create(min_hessian);
    }

    virtual ~FeatureDetection();
	static void foo();

    // the wrapper
    void computeKeypointsAndDraw(char *pathname);
    
    // methods for feature detections, SIFT, FAST and SURF
    void fastDetector(cv::Mat img, std::vector<cv::KeyPoint>& detectedPoints);
    void siftDetector(cv::Mat img, std::vector<cv::KeyPoint>& detectedPoints);
    void surfDetector(cv::Mat img, std::vector<cv::KeyPoint>& detectedPoints);

    // methods for extracting the features
    void siftExtraction (cv::Mat img, std::vector<cv::KeyPoint> detectedPoints, cv::Mat &descriptor);
    void surfExtraction (cv::Mat img, std::vector<cv::KeyPoint> detectedPoints, cv::Mat &descriptor);

    // draw keypoints
    void drawKeypoints (cv::Mat img, std::vector<cv::KeyPoint> detectedPoints, cv::Mat &output);
    
private:
    // pointer to the feature point detector object
    cv::Ptr<cv::FeatureDetector> siftdetect_;
    cv::Ptr<cv::FeatureDetector> fastdetect_;
    cv::Ptr<cv::FeatureDetector> surfdetect_;

    // pointer to the feature descriptor extractor object
    cv::Ptr<cv::DescriptorExtractor> siftextract_;
    cv::Ptr<cv::DescriptorExtractor> surfextract_;

    // pointer to the matcher object
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // fast param
    int fast_threshold;
    bool nonMaxSuppression;

    // surf param
    int min_hessian;
    int octave_layer;
    double contrast_threshold;
    double edge_threshold;
    double sigma;

    // sift param
    float sift_matching_ratio;
};

#endif