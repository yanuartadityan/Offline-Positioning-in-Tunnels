/*
	AUTHOR: DAVID BENNEHAG and YANUAR T. A. NUGRAHA

	Requires OpenCV (tested on 3.1) AND PCL


*/
#include <cvsba.h>
#include <sba.h>

//OPENCV
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

//OUR STUFF
#include "FeatureDetection.h"
#include "PnPSolver.h"
#include "Calibration.h"
//#include "VisualOdometry.h"a
#include "Reprojection.h"


//C++ STUFF
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <fstream>

using namespace std;
using namespace cv;

const int NR_OF_FRAMES = 250;
const int FIRST_INDEX = 433, LAST_INDEX = FIRST_INDEX + NR_OF_FRAMES;

const int NUMBEROFTHREADS = 16;
const bool PAR_MODE = true;

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


void calcBestPoint(
	vector< pair<Point3d, Mat> >& _3dToDescriptorVector,
	Mat T,
	Mat K,
	vector<KeyPoint> keypoints1,
	Mat descriptors1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree,
	int start,
	int end,
	int threadIndex);


void prepareMap(char* mapCoordinateFile, char* mapKeypointsFile, vector<Point2d>& tunnel2D, vector<Point3d>& tunnel3D, Mat& tunnelDescriptor);
//vector< pair<Point3d, Mat> > manualStuff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree);
string type2str(int type);

int main(int argc, char** argv)
{
	ofstream logFile;
	logFile.open("logs/cameraPositions.txt", std::ios::out);

	Mat frame1;
	Mat descriptors1;
	vector<KeyPoint> keypoints1;
	
	auto beginningOfMain = std::chrono::high_resolution_clock::now();

	auto begin = std::chrono::high_resolution_clock::now();
	// We load the point cloud once and then keep it open for the rest of the execution,
	//    since the loading takes alot of time.
	cout << endl << "Loading point cloud... ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("gnistangtunneln-semifull-voxelized.pcd", *cloud);
    

	//Build the kdtree for searching in the point cloud.
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	auto end = std::chrono::high_resolution_clock::now();
	cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" << endl << endl;

	//begin = std::chrono::high_resolution_clock::now();
	// Before running our loop through the image sequence, do the manual stuff of the first image.
	//cout << "Performing manual labour... ";
	//vector< pair<Point3d, Mat> > _3dToDescriptorVector = manualStuff(cloud);
	vector< pair<Point3d, Mat> > _3dToDescriptorVector;
	//end = std::chrono::high_resolution_clock::now();
	//cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" << endl << endl;

	Calibration calib;
	PnPSolver solver1;
	FeatureDetection fdetect;

	// 2. prepare the manual correspondences as a lookup table
	char map2Dto3D[100];
	char mapDescrip[100];
	char nextimage[100];
	vector<Point2d> tunnel2D;
	vector<Point3d> tunnel3D;
	Mat tunnelDescriptor;
	sprintf(map2Dto3D, "ManualCorrespondences.txt");
	sprintf(mapDescrip, "ManualCorrespondences.yml");

	prepareMap(map2Dto3D, mapDescrip, ref(tunnel2D), ref(tunnel3D), ref(tunnelDescriptor));

	for (int h = 0; h < tunnel3D.size(); h++)
	{
		_3dToDescriptorVector.push_back(make_pair(tunnel3D[h], tunnelDescriptor.row(h)));
	}

	int clearingCounter = 0;
	// Don't start the loop on the image we handled manually.
	for (int i = FIRST_INDEX; i < LAST_INDEX; i++)
	{
		//tunnel2D.clear(); tunnel3D.clear();
		
		begin = std::chrono::high_resolution_clock::now();
		sprintf(nextimage, "imageSequence\\img_%05d.png", i);
		cout << "Loading image: " << nextimage << "... ";
		Mat frame1 = imread(nextimage);
		//String filename = "imageSequence\\img_00" + to_string(i) + ".png";
		//cout << "Loading image: " << filename << "... ";
		//frame1 = imread(filename);

		// set the RoI (Region of Interest)
		// this mask is to take only 50% upper part of the image
		Mat img_maskUpperPart = Mat::zeros(frame1.size(), CV_8U);
		Mat img_roiUpperPart(img_maskUpperPart, Rect(0, 0, frame1.cols, frame1.rows * 4 / 5));
		img_roiUpperPart = Scalar(255, 255, 255);
		
		cout << "Done!" << endl;


		begin = std::chrono::high_resolution_clock::now();
		cout << "Running SIFT... ";
		fdetect.siftDetector(frame1, keypoints1, img_maskUpperPart);
		fdetect.siftExtraction(frame1, keypoints1, descriptors1);
		end = std::chrono::high_resolution_clock::now();
		cout << "Done!\tFound " << descriptors1.rows << " descriptors (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "/" << std::chrono::duration_cast<std::chrono::milliseconds>(end - beginningOfMain).count() << " ms)" << endl;

		

		// Take the descriptor mat out of our lookup table for the matching
		Mat tunnelDescriptors;
		for (pair<Point3d, Mat> pair : _3dToDescriptorVector)
		{
			tunnelDescriptors.push_back(pair.second);
			//cout << "pushing back:" << endl << pair.second << endl << endl;
		}
		
		begin = std::chrono::high_resolution_clock::now();
		cout << "Performing matching... ";
		vector<vector<DMatch> > matches;
		fdetect.bfMatcher(descriptors1, tunnelDescriptors, matches);

		// 8. retrieve the matched indices from the descriptor
		vector<int> matchedIndices;
		vector<int> matchedXYZ;
		float dist1 = 0.0f, dist2 = 0.0f;

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
		cout << "Done!\tMatched " << matches.size() << "/" << descriptors1.rows << " descriptors (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "/" << std::chrono::duration_cast<std::chrono::milliseconds>(end - beginningOfMain).count() << " ms)" << endl;

		
		
		


		/*
		*	We collect the 2D points from our keypoints and 3D points from our LUT.
		*		These are then given to our PnP solver.
		*/
		vector<Point2d> retrieved2D;
		vector<Point3d> retrieved3D;
		for (int k = 0; k < matchedIndices.size(); k++)
		{
			retrieved2D.push_back(Point2d(keypoints1[matchedIndices[k]].pt.x, keypoints1[matchedIndices[k]].pt.y));

			retrieved3D.push_back(Point3d(_3dToDescriptorVector[matchedXYZ[k]].first.x, _3dToDescriptorVector[matchedXYZ[k]].first.y, _3dToDescriptorVector[matchedXYZ[k]].first.z));
			//retrieved3D.push_back(Point3d(tunnel3D[matchedXYZ[k]].x, tunnel3D[matchedXYZ[k]].y, tunnel3D[matchedXYZ[k]].z));
			//cout << std::fixed << setprecision(4);
			//cout << "   pushed {" << keypoints1[matchedIndices[i]].pt.x << ", " << keypoints1[matchedIndices[i]].pt.y << "} --> {"
			//	<< tunnel3D[matchedXYZ[i]].x << ", " << tunnel3D[matchedXYZ[i]].y << ", " << tunnel3D[matchedXYZ[i]].z << "}" << endl;
		}
		cout << "Running solver... ";
		solver1.setImagePoints(retrieved2D);
		solver1.setWorldPoints(retrieved3D);
		solver1.run(0);
		cout << "Done!" << endl << endl;
		cout << "Camera Position:" << endl << solver1.getCameraPosition() << endl;
		Mat T = solver1.getCameraPose().clone();
		Mat K = calib.getCameraMatrix();



		/*
		*	To prevent the LUT from growing too big and possibly match against too old entries,
		*		we periodically clear the first half.
		*/
		vector< pair<Point3d, Mat> > tempLUT;
		vector< pair<Point3d, Mat> >::iterator halfwayItr = _3dToDescriptorVector.begin() + _3dToDescriptorVector.size() / 2;
		vector< pair<Point3d, Mat> >::iterator endItr = _3dToDescriptorVector.end();
		clearingCounter++;
		if (clearingCounter == 5)
		{
			cout << "Clearing the LUT..." << endl;
			clearingCounter = 0;

			tempLUT.insert(tempLUT.end(),
				make_move_iterator(halfwayItr),
				make_move_iterator(endItr));

			_3dToDescriptorVector.clear();

			_3dToDescriptorVector = move(tempLUT);

			tempLUT.clear();

			cout << "New size of LUT: " << _3dToDescriptorVector.size() << endl;
		}



		// Keep track of all our threads
		vector<thread> workers;
		int workerCount = 0;

		begin = std::chrono::high_resolution_clock::now();
		cout << "Performing backprojection of " << descriptors1.rows << " descriptors... ";
		/*
		 *	After finding our projection matrix, we can backproject the descriptors we found
		 *		onto the point cloud.
		 */
		if(PAR_MODE)		//RUN THE BACKPROJECTION WITH THREADS IN PARALLELL
		{
			cout << "Running multithreaded..." << endl;

			int keypointsPerThread = keypoints1.size() / NUMBEROFTHREADS;

			for (int threadIndex = 0; threadIndex < NUMBEROFTHREADS; threadIndex++)
			{
				workerCount++;

				int start = threadIndex * keypointsPerThread;
				int end = (threadIndex + 1) * keypointsPerThread;

				// We create one worker for each keypoint.
				// The order in which they push their results into the look up table does not matter.
				workers.push_back(
						thread( &calcBestPoint,
								ref(_3dToDescriptorVector),
								T,
								K,
								keypoints1,
								descriptors1,
								cloud,
								kdtree,
								start,
								end,
								threadIndex));
			}
			// Join acts as a "wall", so that all threads finish before the main thread continues.
			for (int l = 0; l < workers.size(); l++)
			{
				//cout << "Joining thread #" << workers[l].get_id() << endl;
				if (workers[l].joinable())
					workers[l].join();
			}
			workers.clear();

		}


		else		// RUN IN SEQUENTIAL MODE INSTEAD
		{
			cout << "Running singlethreaded..." << endl;
			for (int counter = 0; counter < keypoints1.size(); counter = counter + 20)
			{
				vector<double> bestPoint = Reprojection::backproject(
					T,
					K,
					Point2d(keypoints1[counter].pt.x, keypoints1[counter].pt.y),
					cloud,
					kdtree);

				if (bestPoint[0] == 0 || bestPoint[1] == 0 || bestPoint[2] == 0)
				{	
					cout << "Threw away empty projection result!" << endl;
					continue;
				}

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
					continue;
				desc = descriptors1.row(counter);

				// Push the pair into the lookup table
				_3dToDescriptorVector.push_back(make_pair(_3dcoord, desc));
				//tunnel3D.push_back(_3dcoord);
				//tunnelDescriptor.push_back(descriptors1.row(counter));
			}
		}
		//END OF BACKPROJECTION
		
		end = std::chrono::high_resolution_clock::now();
		cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "/" << std::chrono::duration_cast<std::chrono::milliseconds>(end - beginningOfMain).count() << " ms)" << endl << endl;

		cout << "Size of our LUT: " << endl << _3dToDescriptorVector.size() << endl;
		



		
		// Calculate the miss rate
		//Point3d cmp; cmp.x = 0; cmp.y = 0; cmp.z = 0;
		//int misses = 0;
		//cout << "All found points: " << endl;
		//for (pair<Point3d, Mat> item : _3dToDescriptorVector)
		//{
		//	//cout << "[" << item.first.x << ", " << item.first.y << ", " << item.first.z << "]" << endl;
		//	if (item.first == cmp)
		//		misses++;
		//}
		//cout << "Number of misses: " << misses << " (" << ((double)misses / _3dToDescriptorVector.size()) * 100 << "%)" << endl;
		
		cout << "Camera Position:" << endl << solver1.getCameraPosition() << endl;
		

		//vector<Point3d> _3dTemp;
		//for (int m = (_3dToDescriptorVector.size() - workerCount); m <= _3dToDescriptorVector.size(); m++)
		//{
		//	_3dTemp.push_back(_3dToDescriptorVector[m].first); 
		//}
		// 15. check reprojection error of each backprojected world points
		/*
		vector<Point2d> reprojectedPixels;
		projectPoints(_3dTemp,
			solver1.getRotationMatrix(),
			solver1.getTranslationVector(),
			calib.getCameraMatrix(),
			calib.getDistortionCoeffs(),
			reprojectedPixels);

		double repError = 0;
		for (int itx = 0; itx < _1dTemp.size(); itx++)
		{
			double dx, dy;

			dx = pow(abs(reprojectedPixels[itx].x - detectedkpts[_1dTemp[itx]].pt.x), 2);
			dy = pow(abs(reprojectedPixels[itx].y - detectedkpts[_1dTemp[itx]].pt.y), 2);

			repError += sqrt(dx + dy);
		}
		*/

		/*
		auto begin = std::chrono::high_resolution_clock::now();
		cout << "Performing Bundle Adjustment... ";
		vector<Point3d> points;
		for (pair<Point3d, Mat> pair : _3dToDescriptorVector)
			points.push_back(pair.first);
		const vector<vector <Point2d> > imagePoints = { solver1.getImagePoints() };
		const vector<vector<int> > visibility = { vector<int>(imagePoints.size(), 1) };
		vector<Mat> cameraMatrices = { calib.getCameraMatrix() };
		vector<Mat> rotationMatrices = { solver1.getRotationMatrix() };
		vector<Mat> translationMatrices = { solver1.getTranslationVector() };
		vector<Mat> distCoeffMatrices = { calib.getDistortionCoeffs() };

		cvsba::Sba sba;
		sba.run(		points,
						imagePoints,
						visibility,
						cameraMatrices,
						rotationMatrices,
						translationMatrices,
						distCoeffMatrices);
		
		cout << " Done!" << endl;
		auto end = std::chrono::high_resolution_clock::now();
		cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms)" << endl << endl;
		*/
		
		cout << "****************** STARTING OVER ******************" << endl;
	}

	auto endOfMain = std::chrono::high_resolution_clock::now();
	cout << "Done! (" << std::chrono::duration_cast<std::chrono::milliseconds>(endOfMain - beginningOfMain).count() << " ms)" << endl << endl;

	logFile << "Camera Positions:" << endl << endl << "[ " << flush;
	for(Mat pos : solver1.camPositions)
		logFile << setprecision(10) 
				<< pos.at<double>(0, 0) << " " 
				<< pos.at<double>(1, 0) << " " 
				<< pos.at<double>(2, 0) << "; ..." << endl << flush;
	logFile << "];" << endl << endl << flush;
	
	
	
	
	

	
	logFile.close();

	return 0;
}








/*

vector< pair<Point3d, Mat> > manualStuff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree)
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

	for (int counter = 0; counter < imagepoints.size(); counter++)
	{	
		workers.push_back(
			thread(&calcBestPoint,
				&_3dToDescriptorVector,
				T,
				K,
				keypoints1,
				cloud,
				descriptors1,
				counter));

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
*/

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




void prepareMap(char* mapCoordinateFile, char* mapKeypointsFile, vector<Point2d>& tunnel2D, vector<Point3d>& tunnel3D, Mat& tunnelDescriptor)
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

}



void calcBestPoint(
	vector< pair<Point3d, Mat> >& _3dToDescriptorVector,
	Mat T,
	Mat K,
	vector<KeyPoint> keypoints1,
	Mat descriptors1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree,
	int start,
	int end,
	int threadIndex)
{
	//vector<double> bestPoint = Reprojection::backproject(
	//	T,
	//	K,
	//	Point2d(keypoints1[counter].pt.x, keypoints1[counter].pt.y),
	//	cloud);


	for(int i = start; i < end; i++)
	{
		Point2d queryPoint = Point2d(keypoints1[i].pt.x, keypoints1[i].pt.y);

		vector<double> bestPoint = Reprojection::backproject(T, K, queryPoint, cloud, kdtree);

		if (bestPoint[0] == 0 && bestPoint[1] == 0 && bestPoint[2] == 0)
			return;


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
		if (i > descriptors1.rows)
			return;

		desc = descriptors1.row(i);

		// Vectors are not thread safe, make sure only one thread at a time access it.
		global_mutex.lock();
		//cout << "thread " << this_thread::get_id() << " found point " << endl
		//	<< "X = \t" << bestPoint[0] << endl
		//	<< "Y = \t" << bestPoint[1] << endl
		//	<< "Z = \t" << bestPoint[2] << endl
		//	<< "DIST = \t" << bestPoint[3] << endl;
		// Push the pair into the lookup table
		_3dToDescriptorVector.push_back(make_pair(_3dcoord, desc));
		//tunnel3D.push_back(_3dcoord);
		//tunnelDescriptor.push_back(descriptors1.row(counter));
		global_mutex.unlock();
	}
}



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