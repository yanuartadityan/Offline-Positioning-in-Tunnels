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


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	
	

	vector< pair<Point3d, Mat> > _3dToDescriptorVector;

    Calibration calib;
	PnPSolver solver1, solver2;
	VO vodometry;
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

    
    // MANUAL CORRESPONDENCES PNP SOLVER
	cout << "Running first solver... ";

	vector<Point2d> imageOne;
	imageOne.push_back(Point2d(397.210571, 145.146866));	imageOne.push_back(Point2d(650.494934, 129.172379));
	imageOne.push_back(Point2d(519.567688, 131.898239));	imageOne.push_back(Point2d(531.834473, 267.480103));
	imageOne.push_back(Point2d(239.835358, 207.141220));	imageOne.push_back(Point2d(834.740051, 174.580566));
	imageOne.push_back(Point2d(211.190155, 510.402740));	imageOne.push_back(Point2d(437.319458, 218.244186));
	imageOne.push_back(Point2d(845.259948, 160.413910));	imageOne.push_back(Point2d(559.729248, 170.678528));

	solver1.setImagePoints(imageOne);
	solver1.foo(0);
	cout << "Done!" << endl << endl;
	
	cout << "Running second solver... ";

	vector<Point2d> imageTwo;
	imageTwo.push_back(Point2d(490, 250));	imageTwo.push_back(Point2d(668, 242));
	imageTwo.push_back(Point2d(578, 242));	imageTwo.push_back(Point2d(582, 335));
	imageTwo.push_back(Point2d(380, 294));	imageTwo.push_back(Point2d(793, 278));
	imageTwo.push_back(Point2d(367, 499));	imageTwo.push_back(Point2d(521, 306));
	imageTwo.push_back(Point2d(806, 262));	imageTwo.push_back(Point2d(604, 272));

	solver2.setImagePoints(imageTwo);
	solver2.foo(0);
	cout << "Done!" << endl;

	//cout << endl << endl << "camera 1 position: " << endl << solver1.getCameraPosition() << endl << "camera 2 position: " << endl << solver2.getCameraPosition() << endl;

	Mat frame1 = imread("img_00433.png");
	
	vector<KeyPoint> keypoints1;
	fdetect.surfDetector(frame1,keypoints1);

	Mat descriptors1;
	fdetect.siftExtraction(frame1, keypoints1,descriptors1);
	
	
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










	// We load the point cloud once and then keep it open for the rest of the execution,
	//    since the loading takes alot of time.
	cout << endl << "Loading point cloud... ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("cloud.pcd", *cloud);
	cout << "Done!" << endl << endl;





	
	Mat T = solver1.getCameraPose().clone();
	Mat K = calib.getCameraMatrix();

	vector<double> bestPoint{ 0, 0, 0, 1000 };

	vector<Point2d> imagepoints = solver1.getVoVImagePoints()[0];
	
	/*
	*	For every feature point that we find in our image, we do the backprojection.
	*/
	for(int counter = 0; counter < imagepoints.size(); counter++)
	{ 
		
		bestPoint = Reprojection::backproject(T, K, imagepoints[counter], cloud);

		cout << setprecision(15);
		cout << "*****************************" << endl;
		cout << "Seaching for image point\t" << imagepoints[counter] << endl << endl;
		cout << "The best point found:"		 << endl
			<< "X = \t"		<< bestPoint[0]  << endl
			<< "Y = \t"		<< bestPoint[1]  << endl
			<< "Z = \t"		<< bestPoint[2]  << endl
			<< "DIST = \t"	<< bestPoint[3]  << endl;
		cout << "*****************************\n\n\n\n\n";

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

		// Push the pair into the lookup table
		_3dToDescriptorVector.push_back(make_pair(_3dcoord, desc));
		
	}
	



	return 0;
}
