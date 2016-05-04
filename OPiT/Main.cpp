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
#include <chrono>

using namespace std;
using namespace cv;

const int FIRST_INDEX = 433, LAST_INDEX = 438;


std::mutex global_mutex;

//vector<Point2d>             tunnel2D;
//vector<Point3d>             tunnel3D;
//Mat                         tunnelDescriptor;

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

void prepareMap(char* mapCoordinateFile, char* mapKeypointsFile);
vector< pair<Point3d, Mat> > manualStuff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
string type2str(int type);

int main(int argc, char** argv)
{
	Mat frame1;
	Mat descriptors1;
	vector<KeyPoint> keypoints1;

	auto begin = std::chrono::high_resolution_clock::now();
	// We load the point cloud once and then keep it open for the rest of the execution,
	//    since the loading takes alot of time.
	cout << endl << "Loading point cloud... ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("cloud.pcd", *cloud);
	auto end = std::chrono::high_resolution_clock::now();
	cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" << endl << endl;

	begin = std::chrono::high_resolution_clock::now();
	// Before running our loop through the image sequence, do the manual stuff of the first image.
	cout << "Performing manual labour... ";
	vector< pair<Point3d, Mat> > _3dToDescriptorVector = manualStuff(cloud);
	end = std::chrono::high_resolution_clock::now();
	cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" << endl << endl;

    Calibration calib;
	PnPSolver solver1;
    FeatureDetection fdetect;
    
	// 2. prepare the manual correspondences as a lookup table
	//char map2Dto3D[100];
	//char mapDescrip[100];
	//char nextimage[100];
	//sprintf(map2Dto3D, "ManualCorrespondences.txt");
	//sprintf(mapDescrip, "ManualCorrespondences.yml");

	//prepareMap(map2Dto3D, mapDescrip);

	vector<Mat> camPositions;

	// Don't start the loop on the image we handled manually.
	for (int i = FIRST_INDEX + 1; i <= LAST_INDEX; i++)
	{
		//tunnel2D.clear(); tunnel3D.clear();

		begin = std::chrono::high_resolution_clock::now();
		//sprintf(nextimage, "imageSequence\\img_%05d.png", i);
		//Mat frame1 = imread(nextimage);
		String filename = "imageSequence\\img_00" + to_string(i) + ".png";
		cout << "Loading image: " << filename << "... ";
		frame1 = imread(filename);
		cout << "Done!" << endl;

		begin = std::chrono::high_resolution_clock::now();
		cout << "Running SURF/SIFT... ";
		fdetect.siftDetector(frame1, keypoints1);
		fdetect.siftExtraction(frame1, keypoints1, descriptors1);
		end = std::chrono::high_resolution_clock::now();
		cout << "Done!\tFound " << descriptors1.rows << " descriptors (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" <<  endl;
		

		
		//descriptors1.convertTo(descriptors1, CV_32F);
		//cout << "Type of descriptors1 = " << type2str(descriptors1.type()) << endl
		//	<< "Type of tunnelDescriptor = " << type2str(tunnelDescriptor.type()) << endl;


		// Take the descriptor mat out of our lookup table for the matching
		Mat tunnelDescriptor;
		for(pair<Point3d,Mat> pair : _3dToDescriptorVector)
		{
			tunnelDescriptor.push_back(pair.second);
			//cout << "pushing back:" << endl << pair.second << endl << endl;
		}
		
		begin = std::chrono::high_resolution_clock::now();
		cout << "Performing matching... ";
		vector<vector<DMatch> > matches;
		fdetect.bfMatcher(descriptors1, tunnelDescriptor, matches);

		// 8. retrieve the matches indices from the descriptor
		vector<int> matchedIndices;
		vector<int> matchedXYZ;
		float dist1=0.0f, dist2=0.0f;

		for (int j = 0; j < matches.size(); j++)
		{
			DMatch first = matches[j][0];

			dist1 = matches[j][0].distance;
			dist2 = matches[j][1].distance;
			//cout << "dist1 = " << dist1 << "\t dist2 = " << dist2 << endl;
			if (dist1 < fdetect.getSiftMatchingRatio() * dist2)
			{
				matchedIndices.push_back(first.trainIdx);
				matchedXYZ.push_back(first.queryIdx);
			}
		}
		end = std::chrono::high_resolution_clock::now();
		cout << "Done!\tMatched " << matches.size() << "/" << descriptors1.rows << " descriptors (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" << endl;
		

		cout << "Running solver... ";

		vector<Point2d> _2dpoints;

		vector<Point2d> retrieved2D;
		vector<Point3d> retrieved3D;

		//retrieved2D.clear();
		//retrieved3D.clear();

		for (int k = 0; k < matchedIndices.size(); k++)
		{
			retrieved2D.push_back(Point2d(keypoints1[matchedIndices[k]].pt.x, keypoints1[matchedIndices[k]].pt.y));

			retrieved3D.push_back(Point3d(_3dToDescriptorVector[matchedXYZ[k]].first.x, _3dToDescriptorVector[matchedXYZ[k]].first.y, _3dToDescriptorVector[matchedXYZ[k]].first.z));


			//cout << std::fixed << setprecision(4);
			//cout << "   pushed {" << keypoints1[matchedIndices[i]].pt.x << ", " << keypoints1[matchedIndices[i]].pt.y << "} --> {"
			//	<< tunnel3D[matchedXYZ[i]].x << ", " << tunnel3D[matchedXYZ[i]].y << ", " << tunnel3D[matchedXYZ[i]].z << "}" << endl;
		}	

		solver1.setImagePoints(retrieved2D);
		solver1.setWorldPoints(retrieved3D);
		solver1.run(0);
		cout << "Done!" << endl << endl;

		Mat T = solver1.getCameraPose().clone();
		Mat K = calib.getCameraMatrix();
		//vector<Point2d> imagepoints = solver1.getVoVImagePoints()[0];

		// Create a vector of "workers", each doing a separate backproject calculation.
		vector<thread> workers;
		int workerCount = 0;
		//cout << "Keypoints size = " << keypoints1.size() << endl;

		//if(i % 5 == 0)
		//	_3dToDescriptorVector.clear();

		begin = std::chrono::high_resolution_clock::now();
		cout << "Performing backprojection... ";
		/*
		*	For every feature point that we find in our image, we do the backprojection.
		*/
		for (int counter = 0; counter < keypoints1.size(); counter = counter + 50)
		{
			workerCount++;

			// We create one worker for each keypoint.
			// The order in which they push their results into the look up table does not matter.
			workers.push_back(thread([&_3dToDescriptorVector, &T, &K, &keypoints1, &cloud, &descriptors1, counter]()
			{
				vector<double> bestPoint = Reprojection::backproject(
												T, 
												K, 
												Point2d(keypoints1[counter].pt.x, keypoints1[counter].pt.y), 
												cloud);

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
				if (counter > descriptors1.rows)
					return;
				desc = descriptors1.row(counter);

				// Vectors are not thread safe, make sure only one thread at a time access it.
				global_mutex.lock();
				
				// Push the pair into the lookup table
				_3dToDescriptorVector.push_back(make_pair(_3dcoord, desc));
				global_mutex.unlock();
			}));
		}

		// Join acts as a "wall", so that all threads finish before the main thread continues.
		for (int l = 0; l < workers.size(); l++)
		{
			//cout << "Joining thread #" << workers[i].get_id() << endl;
			if (workers[l].joinable())
			{
				//workerCount--;
				workers[l].join();
				//cout << workerCount << " workers remaining!" << endl;
			}
		}
		end = std::chrono::high_resolution_clock::now();
		cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" << endl << endl;

		// Calculate the miss rate
		Point3d cmp; cmp.x = 0; cmp.y = 0; cmp.z = 0;
		int misses = 0;

		cout << "Size of our LUT: " << endl << _3dToDescriptorVector.size() << endl;
		cout << "All found points: " << endl;
		for (pair<Point3d, Mat> item : _3dToDescriptorVector)
		{
			cout << "[" << item.first.x << ", " << item.first.y << ", " << item.first.z << "]" << endl;
			if (item.first == cmp)
				misses++;
		}
		cout << "Number of misses: " << misses << " (" << ((double)misses / _3dToDescriptorVector.size())*100 << "%)" << endl;
		
		cout << "Camera Position:" << endl << solver1.getCameraPosition() << endl;
		camPositions.push_back(solver1.getCameraPosition());
		

		cout << "****************** STARTING OVER ******************" << endl << endl;
	}

	cout << "Camera Positions:" << endl;
	for(Mat pos : camPositions)
		 cout << pos << endl;

	
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
	//cout << "Running first solver... ";

	vector<Point2d> imageOne;
	imageOne.push_back(Point2d(397.210571, 145.146866));	imageOne.push_back(Point2d(650.494934, 129.172379));
	imageOne.push_back(Point2d(519.567688, 131.898239));	imageOne.push_back(Point2d(531.834473, 267.480103));
	imageOne.push_back(Point2d(239.835358, 207.141220));	imageOne.push_back(Point2d(834.740051, 174.580566));
	imageOne.push_back(Point2d(211.190155, 510.402740));	imageOne.push_back(Point2d(437.319458, 218.244186));
	imageOne.push_back(Point2d(845.259948, 160.413910));	imageOne.push_back(Point2d(559.729248, 170.678528));

	solver1.setImagePoints(imageOne);
	solver1.run(1);
	//cout << "Done!" << endl << endl;

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

	// This is the image we did manual matching on
	frame1 = imread("img_00433.png");
	
	//cout << "Running SURF/SIFT... ";
	fdetect.siftDetector(frame1, keypoints1);
	fdetect.siftExtraction(frame1, keypoints1, descriptors1);
	//cout << "Done!" << endl;

	vector<int> indexingVector;
	
	for (int i = 0; i < keypoints1.size(); i++)
	{
		for (int j = 0; j<imageOne.size(); j++)
		{
			if (areSame(i, imageOne[j].x, imageOne[j].y, keypoints1[i].pt.x, keypoints1[i].pt.y))
			{
				//cout << i << endl;
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
	for (int counter = 0; counter < imagepoints.size(); counter++)
	{	
		workers.push_back(thread([&_3dToDescriptorVector,&T, &K, &imagepoints, &cloud, &descriptors1, &indexingVector, counter]()
		{
			vector<double> bestPoint = Reprojection::backproject(T, K, imagepoints[counter], cloud);

			
			// Define the 3D coordinate
			Point3d _3dcoord; _3dcoord.x = bestPoint[0]; _3dcoord.y = bestPoint[1]; _3dcoord.z = bestPoint[2];

			// Define its descriptor, should have size 1x128
			Mat desc;
			
			desc = descriptors1.row( indexingVector[counter] );


			// Vectors are not thread safe, make sure only one thread at a time access it.
			global_mutex.lock();
			//cout << counter << endl;
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

	//cout << "Size of our LUT: " << endl << _3dToDescriptorVector.size() << endl;

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

/*
void prepareMap(char* mapCoordinateFile, char* mapKeypointsFile)
{
	// load descriptor
	string line;
	ifstream mapFile;
	mapFile.open(mapCoordinateFile);

	double temp = 0, a = 0, b = 0, x = 0, y = 0, z = 0;

	if (mapFile.is_open())
	{
		while (getline(mapFile, line) && !mapFile.eof())
		{
			istringstream in(line);

			for (int i = 0; i<5; i++)
			{
				in >> temp;

				if (i == 0)
					a = temp;
				if (i == 1)
					b = temp;
				if (i == 2)
					x = temp;
				if (i == 3)
					y = temp;
				if (i == 4)
					z = temp;
			}
			tunnel2D.push_back(Point2f(a, b));
			tunnel3D.push_back(Point3f(x, y, z));
		}
	}
	mapFile.close();

	// load keypoints
	cv::FileStorage lstorage(mapKeypointsFile, cv::FileStorage::READ);
	lstorage["img"] >> tunnelDescriptor;
	lstorage.release();

}*/