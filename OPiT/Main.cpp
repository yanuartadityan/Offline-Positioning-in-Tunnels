/*
	AUTHOR: DAVID BENNEHAG and YANUAR T. A. NUGRAHA

	Requires OpenCV (tested on 3.1) AND PCL


*/
//OPENCV
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//OUR STUFF
#include "FeatureDetection.h"
#include "PnPSolver.h"
#include "Calibration.h"
#include "VisualOdometry.h"
#include "Reprojection.h"


//C++ STUFF
#include <iostream>
#include <string>
#include <thread>
#include <mutex>



using namespace std;
using namespace cv;

int global_threadcount = 0;
std::mutex global_mutex;

#define EPSILON 0.00001
int areSame(int index, double x, double y, double xx, double yy)
{
	if ((fabs(x - xx) < EPSILON) && (fabs(y - yy) < EPSILON))
	{
		return true;
	}
	else
		return false;
}

vector< pair<Point3d, Mat> > manualStuff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
string type2str(int type);

int main(int argc, char** argv)
{
	Mat frame1;
	Mat descriptors1;
	vector<KeyPoint> keypoints1;


	// We load the point cloud once and then keep it open for the rest of the execution,
	//    since the loading takes alot of time.
	cout << endl << "Loading point cloud... ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("cloud.pcd", *cloud);
	cout << "Done!" << endl << endl;

	// Before running our loop through the image sequence, do the manual stuff of the first image.
	vector< pair<Point3d, Mat> > _3dToDescriptorVector = manualStuff(cloud);
	cout << "Manual stuff done!" << endl << endl;

    Calibration calib;
	PnPSolver solver1, solver2;
	//VO vodometry;
    FeatureDetection fdetect;
    
    Mat distCoeffs = calib.getDistortionCoeffs();



#ifndef KITTI_DATASET
    char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/volvo/out0/";
#else
    char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/kitti-odometry/dataset/sequences/00/image_0/";
#endif
    
    // CHECKING THE IMAGE QUALITY PER DIFFERENT FEATURE DETECTION
//    fdetect.computeKeypointsAndDraw(pathname);
    
    // VISUAL ODOMETRY
    //vodometry.initParameter();
    //vodometry.setImagePath(pathname);
    //vodometry.visualodometry();

	const int FIRST_INDEX = 433, LAST_INDEX = 443;

	// Don't start the loop on the image we handled manually.
	for (int i = FIRST_INDEX + 1; i <= LAST_INDEX; i++)
	{
		frame1 = imread("img_00" + to_string(i) + ".png");

		cout << "Running SURF/SIFT... ";
		fdetect.siftDetector(frame1, keypoints1);
		fdetect.siftExtraction(frame1, keypoints1, descriptors1);
		cout << "Done!" << endl;

		cout << "Running solver... ";

		vector<Point2f> _2dpoints;

		Mat tunnelDescriptor;

		for (int i = 0; i < _3dToDescriptorVector.size(); i++)
		{	
			tunnelDescriptor.push_back(_3dToDescriptorVector[i].second);
		}
		//cout << _3dToDescriptorVector[0].second.cols;

		tunnelDescriptor.convertTo(tunnelDescriptor, CV_8UC1);
		cout << endl << "descriptors1: " << type2str(descriptors1.type()) << endl << "tunnelDescriptor: " << type2str(tunnelDescriptor.type()) << endl << endl;

		vector<vector<DMatch> > matches;
		fdetect.bfMatcher(descriptors1, tunnelDescriptor, matches);
		

		solver1.setImagePoints(_2dpoints);
		solver1.run(0);
		cout << "Done!" << endl << endl;

		//cout << endl << endl << "camera 1 position: " << endl << solver1.getCameraPosition() << endl << "camera 2 position: " << endl << solver2.getCameraPosition() << endl;


	}


	//SIFT WAS HERE



	
	/*
	// Skeleton code for iterating through the image sequence
	while (vc.read(frame2))
	{

	// Change brightness
	//frame2 = frame2 + Scalar(10,10,10);


	//resize(frame2, frame2, Size(frame2.cols / 2, frame2.rows / 2));

	//In the first iteration, only frame2 will contain a frame, so skip this
	if (counter == 0)
	{
	counter++;
	frame1 = frame2.clone();
	continue;
	}

	counter++;

	// frame1 will hold the previous frame, and in the next iteration we will read a new frame into frame2
	//    These two will thus be continuously cycled.
	frame1 = frame2.clone();

	if (waitKey(5000) == 'k')
	break;
	}
	*/

	
	
	Mat T = solver1.getCameraPose().clone();
	Mat K = calib.getCameraMatrix();
	vector<Point2d> imagepoints = solver1.getVoVImagePoints()[0];
	
	// Create a vector of "workers", each doing a separate backproject calculation.
	vector<thread> workers;

	/*
	*	For every feature point that we find in our image, we do the backprojection.
	*/
	for (int counter = 0; counter < imagepoints.size(); counter++)
	{
		workers.push_back(thread([&] ()
		{
			vector<double> bestPoint = Reprojection::backproject(T, K, imagepoints[counter], cloud);

			//cout << setprecision(15);
			//cout << "*****************************" << endl;
			//cout << "Seaching for image point\t" << imagepoints[counter] << endl << endl;
			//cout << "The best point found:" << endl
			//	<< "X = \t" << bestPoint[0] << endl
			//	<< "Y = \t" << bestPoint[1] << endl
			//	<< "Z = \t" << bestPoint[2] << endl
			//	<< "DIST = \t" << bestPoint[3] << endl;
			//cout << "*****************************\n\n\n\n\n";

			/*
			*	Update the Look Up Table for what descriptor belongs to which image point
			*
			*	_3dToDescriptorMap.first[0]  == 3D coordinates vector
			*
			*	_3dToDescriptorVector[i].first.x == X
			*	_3dToDescriptorVector[i].first.y == Y
			*	_3dToDescriptorVector[i].first.z == Z
			*
			*	_3dToDescriptorVector[i].second == its descriptor
			*/
			// Define the 3D coordinate
			Point3d _3dcoord; _3dcoord.x = bestPoint[0]; _3dcoord.y = bestPoint[1]; _3dcoord.z = bestPoint[2];

			// Define its descriptor, should have size 1x128
			Mat desc;
			desc = descriptors1.row(counter);


			// Vectors are not thread safe, make sure only one thread at a time access it.
			global_mutex.lock();
			// Push the pair into the lookup table
			_3dToDescriptorVector.push_back(make_pair(_3dcoord, desc));
			global_mutex.unlock();

		}));

	}
	
	// Join acts as a "wall", so that all threads finish before the main thread continues.
	for (int i = 0; i < workers.size(); i++)
	{
		//cout << "Joining thread #" << workers[i].get_id() << endl;
		if(workers[i].joinable())
			workers[i].join();
	}
	
	cout << "Size of our LUT: " << endl << _3dToDescriptorVector.size() << endl;

	return 0;
}
















vector< pair<Point3d, Mat> > manualStuff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Mat frame1;
	Mat descriptors1, descriptors2;
	vector<KeyPoint> keypoints1, keypoints2;
	vector< pair<Point3d, Mat> > _3dToDescriptorVector;

	Calibration calib;
	PnPSolver solver1, solver2;
	
	FeatureDetection fdetect;

	Mat distCoeffs = calib.getDistortionCoeffs();

	// MANUAL CORRESPONDENCES PNP SOLVER
	cout << "Running first solver... ";

	vector<Point2f> imageOne;
	imageOne.push_back(Point2d(397.210571, 145.146866));	imageOne.push_back(Point2d(650.494934, 129.172379));
	imageOne.push_back(Point2d(519.567688, 131.898239));	imageOne.push_back(Point2d(531.834473, 267.480103));
	imageOne.push_back(Point2d(239.835358, 207.141220));	imageOne.push_back(Point2d(834.740051, 174.580566));
	imageOne.push_back(Point2d(211.190155, 510.402740));	imageOne.push_back(Point2d(437.319458, 218.244186));
	imageOne.push_back(Point2d(845.259948, 160.413910));	imageOne.push_back(Point2d(559.729248, 170.678528));

	solver1.setImagePoints(imageOne);
	solver1.run(0);
	cout << "Done!" << endl << endl;

	//cout << "Running second solver... ";
	//
	//vector<Point2d> imageTwo;
	//imageTwo.push_back(Point2d(490, 250));	imageTwo.push_back(Point2d(668, 242));
	//imageTwo.push_back(Point2d(578, 242));	imageTwo.push_back(Point2d(582, 335));
	//imageTwo.push_back(Point2d(380, 294));	imageTwo.push_back(Point2d(793, 278));
	//imageTwo.push_back(Point2d(367, 499));	imageTwo.push_back(Point2d(521, 306));
	//imageTwo.push_back(Point2d(806, 262));	imageTwo.push_back(Point2d(604, 272));
	//
	//solver2.setImagePoints(imageTwo);
	//solver2.foo(0);
	//cout << "Done!" << endl;

	frame1 = imread("img_00433.png");

	cout << "Running SURF/SIFT... ";

	fdetect.siftDetector(frame1, keypoints1);

	fdetect.siftExtraction(frame1, keypoints1, descriptors1);
	cout << "Done!" << endl;

	vector<int> indexingVector;
	
	for (int i = 0; i < keypoints1.size(); i++)
	{
		for (int j = 0; j<imageOne.size(); j++)
		{
			if (areSame(i, imageOne[j].x, imageOne[j].y, keypoints1[i].pt.x, keypoints1[i].pt.y))
			{
				cout << i << endl;
				//pxIdx.push_back(kpts[i]);
				indexingVector.push_back(i);
				imageOne.erase(imageOne.begin() + j);
			}
		}
	}


	Mat T = solver1.getCameraPose().clone();
	Mat K = calib.getCameraMatrix();
	vector<Point2d> imagepoints = solver1.getVoVImagePoints()[0];

	// Create a vector of "workers", each doing a separate backproject calculation.
	vector<thread> workers;

	/*
	*	For every feature point that we find in our image, we do the backprojection.
	*/
	for (int counter = 0; counter < indexingVector.size(); counter++)
	{
		workers.push_back(thread([&]()
		{
			vector<double> bestPoint = Reprojection::backproject(T, K, imagepoints[counter], cloud);

			
			// Define the 3D coordinate
			Point3d _3dcoord; _3dcoord.x = bestPoint[0]; _3dcoord.y = bestPoint[1]; _3dcoord.z = bestPoint[2];

			// Define its descriptor, should have size 1x128
			Mat desc;
			desc = descriptors1.row( indexingVector[counter] );


			// Vectors are not thread safe, make sure only one thread at a time access it.
			global_mutex.lock();
			// Push the pair into the lookup table
			_3dToDescriptorVector.push_back(make_pair(_3dcoord, desc));
			global_mutex.unlock();

		}));

	}

	// Join acts as a "wall", so that all threads finish before the main thread continues.
	for (int i = 0; i < workers.size(); i++)
	{
		//cout << "Joining thread #" << workers[i].get_id() << endl;
		if (workers[i].joinable())
			workers[i].join();
	}

	cout << "Size of our LUT: " << endl << _3dToDescriptorVector.size() << endl;

	return _3dToDescriptorVector;
}

string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}