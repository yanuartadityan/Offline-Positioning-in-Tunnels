#include "SIFTdetector.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>


#include <stdlib.h>     //for using the function sleep
#include <iostream>
#include <thread>
#include <iomanip>

using namespace cv;
using namespace std;

//Handy function for finding type of image
string type2str(int type);

/*
	Each detect() + compute() of SIFT takes ~3.5 seconds, possible to improve?
*/
void SIFTdetector::foo()
{
	BFMatcher matcher;
	std::vector<DMatch> matches, goodMatches;
	double max_dist = 0, min_dist = 100;

	Ptr<Feature2D> sfd = xfeatures2d::SIFT::create(
		0,			// int 		nfeatures
		3,			// int 		nOctaveLayers
		0.04,		// double 	contrastThreshold
		10,			// double 	edgeThreshold
		1.6			// double 	sigma
		);
	

	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2, imgKeypoints1, imgKeypoints2;

	// Detect keypoints in the frame
	//sfd->detect(frame2, keypoints2);
		
	// Compute the 128 dimension SIFT descriptor at each keypoint.
	// Each row in "descriptors" correspond to the SIFT descriptor for each keypoint
	//sfd->compute(frame2, keypoints2, descriptors2);

	//drawKeypoints(frame2, keypoints2, imgKeypoints2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
		
	//sfd->detect(frame1, keypoints1);
	//sfd->compute(frame1, keypoints1, descriptors1);
	
	
}



SIFTdetector::SIFTdetector()
{
	foo();

	exit(EXIT_SUCCESS);
}




