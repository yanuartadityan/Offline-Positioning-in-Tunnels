#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cvsba/cvsba.h>
#include "FeatureDetection.h"
#include "Calibration.h"
#include "PnPSolver.h"
#include "PointProjection.h"
#include "PCLCloudSearch.h"
#include "Reprojection.h"
#include "Common.h"
#include <iostream>
#include <fstream>
#include <numeric>
#include <thread>
#include <chrono>
#include <mutex>
#define STARTIDX          433
#define FINISHIDX         523
#define SLIDINGWINDOWSIZE 5
#define NUMTHREADS        16
#define STATICNTASK       480
#define DRAWKPTS          1
#define SEQMODE           0
using namespace std;
using namespace cv;
using namespace pcl;
using namespace std::chrono;
vector<Point2d>        tunnel2D;
vector<Point3d>        tunnel3D;
vector<Point3d>        _3dTemp;
vector<Point2d>        _2dTemp;
vector<int>            _1dTemp;
vector<int>            _slidingWindowSize;
Mat                    tunnelDescriptor;
mutex                  g_mutex;
bool                   ransacmode = true;
/* --------------------------------------threadc++11----------------------------------------*/
void mpThread ( Mat T, Mat K, vector<KeyPoint> imagepoint, Mat descriptor,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree,
                int start, int end, int tidx)
{   vector <double> temp = {0,0,0,1000};
    Point3d _mp3dcoord;
    for (int i=start; i<end; i+=4)
    {
        temp = Reprojection::backproject(T, K, Point2d(imagepoint[i].pt.x,imagepoint[i].pt.y), cloud, kdtree);
        _mp3dcoord.x = temp[0]; _mp3dcoord.y = temp[1]; _mp3dcoord.z = temp[2];
        if ((_mp3dcoord.x > 0.0f) && (_mp3dcoord.y > 0.0f) && (_mp3dcoord.z > 0.0f))
        {
            lock_guard<mutex> lock(g_mutex);
            {
                tunnel2D.push_back(Point2d(imagepoint[i].pt.x,imagepoint[i].pt.y));
                tunnel3D.push_back(_mp3dcoord);
                tunnelDescriptor.push_back(descriptor.row(i));
                _3dTemp.push_back(_mp3dcoord);
                _2dTemp.push_back(Point2d(imagepoint[i].pt.x,imagepoint[i].pt.y));
                _1dTemp.push_back(i);
            }
        }
    }
}
/* -------------------------------------themaincaller---------------------------------------*/
int main (int argc, char *argv[])
{
    int startFrame, lastFrame;
    if (argc == 3) {startFrame = atoi(argv[1]);lastFrame  = atoi(argv[2]);}
    else {startFrame = STARTIDX;lastFrame = FINISHIDX;}
    Calibration cal;
    PnPSolver pnp(1000, 5, 0.99);
    PnPSolver pnprefined(1000, 1, 0.99);
    FeatureDetection feat;
    Common com;
    char map2Dto3D  [100]; char mapDescrip [100];
    sprintf(map2Dto3D, "/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT/ManualCorrespondences.txt");
    sprintf(mapDescrip,"/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT/ManualCorrespondences.yml");
    com.prepareMap(map2Dto3D, mapDescrip, &tunnel2D, &tunnel3D, &tunnelDescriptor);
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    io::loadPCDFile("/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT2/cloud/gnistangtunneln-semifull-voxelized.pcd", *cloud);
    std::cerr 	<< "cloud: " << cloud->width * cloud->height << " points (" << pcl::getFieldsList (*cloud) << ")" << std::endl;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    char pathname[100] = "/Users/januaditya/Thesis/exjobb-data/volvo/out0/";
    char nextimage[100];
    Mat T, K;
    Point3d _3dcoord;
    Mat descTemp;
    namedWindow ("disp", WINDOW_NORMAL);
    int frameIdx = startFrame;
    int frameCounter = 0;
    while (frameIdx < lastFrame)
    {
        com.startTimer();
        if (frameCounter > 0) pnp.setPnPParam(1000, 1, 0.99);
        sprintf(nextimage, "%simg_%05d.png", pathname, frameIdx);
        Mat img = imread(nextimage);
        Mat imgs = img.clone();
        Mat img_maskUpperPart = Mat::zeros(img.size(), CV_8U);
        Mat img_roiUpperPart (img_maskUpperPart, Rect(0, 0, img.cols, img.rows*7/8));
        img_roiUpperPart = Scalar(255, 255, 255);
        vector<KeyPoint> detectedkpts;
        Mat descriptor;
        feat.siftDetector(img, detectedkpts, img_maskUpperPart);
        feat.siftExtraction(img, detectedkpts, descriptor);
        vector<vector<DMatch> > matches;
        feat.bfMatcher(tunnelDescriptor, descriptor, matches);
        vector<int> matchedIndices;
        vector<int> matchedXYZ;
        vector<Point2d> lutPt, framePt;
        vector<DMatch> goodMatches,goodMatches2;
        for (int i = 0; i < matches.size(); ++i)
        {
            DMatch first  = matches[i][0];
            auto dist1 = matches[i][0].distance;
            auto dist2 = matches[i][1].distance;
            if(dist1 < feat.getSiftMatchingRatio() * dist2)
            {
                goodMatches.push_back(first);
                matchedIndices.push_back(first.trainIdx);
                matchedXYZ.push_back(first.queryIdx);
                framePt.push_back(detectedkpts[first.trainIdx].pt);
                lutPt.push_back(tunnel2D[first.queryIdx]);
            }
        }

        // --------------------------------------------------------ransac
        if (ransacmode && frameCounter > 0)
        {
            vector<uchar> state;
            findFundamentalMat(lutPt, framePt, FM_LMEDS, 1, 0.99, state);
            vector<Point2d> lutPtRefined, framePtRefined;
            for (size_t i = 0; i < state.size(); ++i)
            {
                if (state[i] != 0)
                {
                    goodMatches2.push_back(goodMatches[i]);
                    matchedIndices.push_back(goodMatches[i].trainIdx);
                    matchedXYZ.push_back(goodMatches[i].queryIdx);
                    lutPtRefined.push_back(lutPt[i]);
                    framePtRefined.push_back(framePt[i]);
                }
            }
            cout << "processing frame-" << frameIdx << endl;
            cout << "number of RAW matches (SIFT ratio)     : " << goodMatches.size() << endl;
            cout << "number of REF matches (RANSAC)         : " << goodMatches2.size() << endl;
            cout << "number of removed outliers         : " << goodMatches.size()-goodMatches2.size() << endl;
            cout << "outliers in percent (%)            : " << (double)(goodMatches.size()-goodMatches2.size())/(double)goodMatches.size()*100 << endl;
            cout << "inliers in percent (%)             : " << 100 - (double)(goodMatches.size()-goodMatches2.size())/(double)goodMatches.size()*100 << endl;
        }
        // --------------------------------------------------------ransac

        vector<Point2d> retrieved2D;vector<Point3d> retrieved3D;
        retrieved2D.clear(); retrieved3D.clear();
        for (int i = 0; i< matchedIndices.size(); i++)
        {
            retrieved2D.push_back(Point2d(detectedkpts[matchedIndices[i]].pt.x,detectedkpts[matchedIndices[i]].pt.y));
            retrieved3D.push_back(Point3d(tunnel3D[matchedXYZ[i]].x,tunnel3D[matchedXYZ[i]].y,tunnel3D[matchedXYZ[i]].z));
        }
        pnp.setImagePoints(retrieved2D); pnp.setWorldPoints(retrieved3D);pnp.run(1);
        T = pnp.getCameraPose().clone(); K = cal.getCameraMatrix();
        Mat R = pnp.getRotationMatrix(); Mat t = pnp.getTranslationVector();
        tunnel2D.clear(); tunnel3D.clear();tunnelDescriptor.release();
        thread *ts[NUMTHREADS];
        int numtask = floor(detectedkpts.size()/NUMTHREADS);
        for (int tidx = 0; tidx < NUMTHREADS; tidx++)
        {
            int start = tidx    * numtask;
            int end   = (tidx+1)* numtask;
            ts[tidx] = new thread (mpThread, T, K, detectedkpts, descriptor, cloud, kdtree, start, end, tidx);
        }
        for (int tidx = 0; tidx < NUMTHREADS; tidx ++) {ts[tidx]->join();}
        pnprefined.setImagePoints(_2dTemp); pnprefined.setWorldPoints(_3dTemp); pnprefined.run(1);
        vector<Point2d> reprojectedPixels;
        projectPoints(_3dTemp,
                      pnp.getRotationMatrix(),
                      pnp.getTranslationVector(),
                      cal.getCameraMatrix(),
                      cal.getDistortionCoeffs(),
                      reprojectedPixels);
        double repError = 0;
        for (int itx = 0; itx < _1dTemp.size(); itx++)
        {
            double dx, dy;
            dx = pow(abs(reprojectedPixels[itx].x - detectedkpts[_1dTemp[itx]].pt.x), 2);
            dy = pow(abs(reprojectedPixels[itx].y - detectedkpts[_1dTemp[itx]].pt.y), 2);
            repError += sqrt(dx + dy);
        }
        cout << "  lut size is " << tunnel3D.size() << endl;
        cout << "  avg reprojection error for " << _1dTemp.size() << " points is: " << repError/_1dTemp.size() << " px " << endl;
        _1dTemp.clear(); _2dTemp.clear(); _3dTemp.clear();
        frameIdx++; frameCounter++;
        com.reportTimer();
    }
    return 0;
}
