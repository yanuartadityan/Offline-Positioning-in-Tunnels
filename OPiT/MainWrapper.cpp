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
#include "Calibration.h"
#include "PnPSolver.h"
#include "PointProjection.h"
#include "PCLCloudSearch.h"
#include "Reprojection.h"

#include <iostream>
#include <numeric>
#include <thread>
#include <future>
#include <chrono>
#include <mutex>

#define STARTIDX    433
#define FINISHIDX   453
#define NUMTHREADS  8
#define NUMTASK     64/NUMTHREADS

using namespace std;
using namespace cv;
using namespace pcl;
using namespace std::chrono;

// global variable for tunnel GPS mapping
//unordered_map<Mat, Point3f> tunnelLut;
vector<Point2d>             tunnel2D;
vector<Point3d>             tunnel3D;
Mat                         tunnelDescriptor;
mutex                  g_mutex;

void threadTest (void);
void printTest (int idx);
void prepareMap (char* mapCoordinateFile, char* mapKeypointsFile);
void mpThread (Mat T, Mat K, vector<KeyPoint> imagepoint, Mat descriptor, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start, int end, int tidx);

int MainWrapper()
{
    //threadTest();

    //Calibration moved to its own class.
    Calibration calib;
	PnPSolver solver;
    FeatureDetection fdetect;

    // 1. load pointcloud
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    io::loadPCDFile("entranceTunnelBig.pcd", *cloud);

    // 2. prepare the manual correspondences as a lookup table
    char map2Dto3D  [100];
    char mapDescrip [100];

    sprintf(map2Dto3D, "/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT/ManualCorrespondences.txt");
    sprintf(mapDescrip,"/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT/ManualCorrespondences.yml");

    prepareMap(map2Dto3D, mapDescrip);

    // 3. start the routing and initiate all variables
    char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/volvo/out0/";
    char nextimage[100];

    Mat T, K;
    Point3d _3dcoord;
    Mat descTemp;
    vector<Point3d> _3dTemp;

    int idx = STARTIDX;
    while (idx < STARTIDX+30)
    {
        // start timer
        high_resolution_clock::time_point t1, t2;
        t1 = high_resolution_clock::now();

        cout << "processing image-" << idx << "...";

        // 4.pre. clear tunnel2D
        tunnel2D.clear();

        // 4.pre. clear _3dTemp
        _3dTemp.clear();

        // 4. load images
        sprintf(nextimage, "%simg_%05d.png", pathname, idx);
        Mat img = imread(nextimage);

        // 5. perform sift feature detection and get kpt_i
        vector<KeyPoint> detectedkpts;
        fdetect.siftDetector(img, detectedkpts);

        // 6. perform sift feature extraction and get desc_i
        Mat descriptor;
        fdetect.siftExtraction(img, detectedkpts, descriptor);

        // 7. match between
        vector<vector<DMatch> > matches;
        fdetect.bfMatcher(descriptor, tunnelDescriptor, matches);

        // 7.1. clear tunnelDescriptor
        cout << "releasing " << tunnel3D.size() << " LUT entries" << endl;
        tunnelDescriptor.release();
        tunnel3D.clear();

        // 8. retrieve the matches indices from the descriptor
        vector<int> matchedIndices;
        vector<int> matchedXYZ;

        for (int i = 0; i < matches.size(); ++i)
        {
            DMatch first  = matches[i][0];

            auto dist1 = matches[i][0].distance;
            auto dist2 = matches[i][1].distance;

            if(dist1 < fdetect.getSiftMatchingRatio() * dist2)
            {
                matchedIndices.push_back(first.trainIdx);
                matchedXYZ.push_back(first.queryIdx);
            }
        }

		cout << setprecision(10);
        cout << "found " << matchedXYZ.size() << " matches with the LUT" << endl;

        // 9. use the indices to retrieve the XYZ from LUT
        vector<Point2d> retrieved2D;
        vector<Point3d> retrieved3D;

        retrieved2D.clear();
        retrieved3D.clear();

        for (int i = 0; i< matchedIndices.size(); i++)
        {
            retrieved2D.push_back(Point2d(detectedkpts[matchedIndices[i]].pt.x,detectedkpts[matchedIndices[i]].pt.y));
            retrieved3D.push_back(Point3d(tunnel3D[matchedXYZ[i]].x,tunnel3D[matchedXYZ[i]].y,tunnel3D[matchedXYZ[i]].z));
            cout << std::fixed << setprecision(4);
            cout << "   pushed {" << detectedkpts[matchedIndices[i]].pt.x << ", " << detectedkpts[matchedIndices[i]].pt.y << "} --> {"
                                  << tunnel3D[matchedXYZ[i]].x << ", " << tunnel3D[matchedXYZ[i]].y << ", " << tunnel3D[matchedXYZ[i]].z << "}" << endl;
        }

        // 10. solvePNP using the XYZ
        solver.setImagePoints(retrieved2D);
        solver.setWorldPoints(retrieved3D);
        solver.foo(1);

        // 11. reproject all 2D keypoints to 3D
        T = solver.getCameraPose().clone();
        K = calib.getCameraMatrix();

        // vector<double> bestPoint{ 0, 0, 0, 1000 };
        // int tempCount = 0;
        // // sequential
        // for(int counter = 0; counter < 32/*detectedkpts.size()/4*/; counter++)
        // {
        //     cout << "  " << counter << "-th step: ";
        //
        //     Point2d queryPoints = Point2d(detectedkpts[counter].pt.x, detectedkpts[counter].pt.y);
        //
        //     cout << "backprojecting " << "(" << queryPoints.x << "," << queryPoints.y << ")" << "...";
        //     bestPoint = Reprojection::backproject(T, K, queryPoints, cloud);
        //
        //     // Define the 3D coordinate
        //     _3dcoord.x = bestPoint[0];
        //     _3dcoord.y = bestPoint[1];
        //     _3dcoord.z = bestPoint[2];
        //
        //     // Define its descriptor, should have size 1x128
        //     descTemp = descriptor.row(counter);
        //
        //     // Push the pair into the lookup table if it's not zero
        //     if ((_3dcoord.x > 0.0f) && (_3dcoord.y > 0.0f) && (_3dcoord.z > 0.0f))
        //     {
        //         tempCount++;
        //         cout << "found a point...(" << tempCount << ")" << endl;
        //
        //         // 12. Update the LUT
        //         tunnel3D.push_back(_3dcoord);
        //         tunnelDescriptor.push_back(descTemp);
        //
        //         // For verifying PnP
        //         _3dTemp.push_back(_3dcoord);
        //         tunnel2D.push_back(queryPoints);
        //     }
        //     else
        //         cout << "nothing found..." << endl;
        // }

        // parallel
        thread *ts[NUMTHREADS];

        for (int tidx = 0; tidx < NUMTHREADS; tidx++)
        {
            int start = tidx    * NUMTASK;
            int end   = (tidx+1)* NUMTASK;

            // spawn threads
            ts[tidx] = new thread (mpThread, T, K, detectedkpts, descriptor, cloud, start, end, tidx);
        }

        for (int tidx = 0; tidx < NUMTHREADS; tidx ++)
        {
            ts[tidx]->join();
        }

        //redo the pnp solver
        /*cout << "now solving again using " << tunnelDescriptor.size() << " descriptors ";
        cout << "and using " << tunnel2D.size() << " keypoints" << endl;
        solver.setImagePoints(tunnel2D);
        solver.setWorldPoints(_3dTemp);
        solver.foo(1);
        cout << tunnel2D << endl;
        cout << _3dTemp << endl;*/

        //increase the idx
        idx++;

        //report the exec
        t2 = high_resolution_clock::now();
        auto duration1 = duration_cast<milliseconds>( t2 - t1 ).count();
        cout << "finish in " << duration1 << "ms (" << duration1/1000 << " sec)" << endl;
    }

    return 1;
}

void prepareMap (char* mapCoordinateFile, char* mapKeypointsFile)
{
    // load descriptor
    string line;
    ifstream mapFile;
    mapFile.open(mapCoordinateFile);

    double temp=0,a=0,b=0,x=0,y=0,z=0;

    if (mapFile.is_open())
    {
        while (getline(mapFile, line) && !mapFile.eof())
        {
            istringstream in(line);

            for (int i=0; i<5; i++)
            {
                in >> temp;

                if (i==0)
                    a = temp;
                if (i==1)
                    b = temp;
                if (i==2)
                    x = temp;
                if (i==3)
                    y = temp;
                if (i==4)
                    z = temp;
            }
            tunnel2D.push_back(Point2f(a,b));
            tunnel3D.push_back(Point3f(x,y,z));
        }
    }
    mapFile.close();

    // load keypoints
    cv::FileStorage lstorage(mapKeypointsFile, cv::FileStorage::READ);
    lstorage["img"] >> tunnelDescriptor;
    lstorage.release();

}

void threadTest (void)
{
    thread *t = new thread[NUMTHREADS];

    for (int i = 0; i < NUMTHREADS; i++)
    {
        t[i] = thread (printTest, i);
    }

    cout << "Launched from the main" << endl;

    for (int i = 0; i < NUMTHREADS; i++)
    {
        t[i].join();
    }
}

void printTest (int idx)
{
    cout << "done by t-id: " << idx << endl;
    sleep(1);
}

void mpThread (Mat T, Mat K, vector<KeyPoint> imagepoint, Mat descriptor, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start, int end, int tidx)
{
    vector <double> temp = {0,0,0,1000};
    Point3d _mp3dcoord;
    Mat _mpDescTemp;
    int tempCount = 0;
    int index;

    for (int i=start; i<end; i++)
    {
        index = tidx*NUMTASK+i;
        temp = Reprojection::backproject(T, K, Point2d(imagepoint[index].pt.x,imagepoint[index].pt.y), cloud);

        // Define the 3D coordinate
        _mp3dcoord.x = temp[0];
        _mp3dcoord.y = temp[1];
        _mp3dcoord.z = temp[2];

        // Define its descriptor, should have size 1x128
        _mpDescTemp = descriptor.row(index);

        // Push the pair into the lookup table if it's not zero
        if ((_mp3dcoord.x > 0.0f) && (_mp3dcoord.y > 0.0f) && (_mp3dcoord.z > 0.0f))
        {
            tempCount++;
            cout << "      tidx-" << tidx << " - found a point at px-" << i << "(" << tempCount << ")" << endl;

            // 12. Update the LUT
            lock_guard<mutex> lock(g_mutex);
            tunnel3D.push_back(_mp3dcoord);
            tunnelDescriptor.push_back(_mpDescTemp);
        }
        else
            cout << "      tidx-" << tidx << " - nothing found..." << endl;
    }
}
