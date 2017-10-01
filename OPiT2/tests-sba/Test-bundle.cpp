// Test-bundle.cpp
//  a main .cpp file for testing whole solution and bundle adjustment at once
#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <numeric>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//  include all class files
#include "Common.h"
#include "Frame.h"
#include "FeatureDetection.h"
#include "PnPSolver.h"
#include "Calibration.h"
#include "BundleAdjust.h"

//  all definitions of variables
#define WINDOWSIZE              1                       // number of tracked frames every timestep
#define NUMTHREADS              8                       // number of threads spawned for backprojections
#define PNPITERATION            1000                    // number of iteration for pnp solver
#define PNPPIXELERROR           10                      // toleration of error pin pixel square
#define PNPCONFIDENCE           0.99                    // confidence level of 99%
#define LENGTHFRAME             100                     // number of processed frames
#define MINFRAMEIDX             433                     // default frame index
#define MAXFRAMEIDX             MINFRAMEIDX+LENGTHFRAME // default frame index + length
#define MINCORRESPONDENCES      3                       // minimum amount of 3D-to-2D correspondencs of PnP
#define THRESHOLD_3DTO2D        200

//  all namespaces
using namespace std;
using namespace cv;
using namespace pcl;

// path to the cloud, tunnel initial correspondences and image sequences
const string cloudPath   = "/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT2/cloud/gnistangtunneln-full.pcd";
const string map2Dto3D   = "/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT/ManualCorrespondences.txt";
const string mapDesc     = "/Users/januaditya/Thesis/exjobb-data/git/Offline-Positioning-in-Tunnels/OPiT/ManualCorrespondences.yml";
const string imgPath     = "/Users/januaditya/Thesis/exjobb-data/volvo/tunnel-frames/";
const string imgPath2    = "/Users/januaditya/Thesis/exjobb-data/gopro/frames/";

bool GoPro               = false;

// helper functions declaration
void drawFeat(cv::Mat img, std::vector<cv::Point2d>& points1, std::vector<cv::Point2d>& points2, int fidx);

//  start the main
int main (int argc, char* argv[])
{
    // declare all the class objects
    Common com;
    FeatureDetection fdet;
    PnPSolver solver;
    Calibration cal;
    BundleAdjust bundle;

    // declare all variables for the lookuptable. it contains vector of pairs between 3D points and their SIFT descriptors
    vector<pair<Point3d, Mat> >     _3dToDescriptorTable;

    // declare all variables for local frame information
    vector<Point3d>                 _tunnel3D;
    vector<Point2d>                 _tunnel2D;
    Mat                             _tunnelDescriptor;

    // set class objects initial parameters
    solver.setPnPParam (PNPITERATION, PNPPIXELERROR, PNPCONFIDENCE);

    // point cloud variables
    int startFrame, endFrame;                                   // marks index for frame start/end
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);  // pointer to the cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    // log
    ofstream logFile, logMatrix, correspondences, correspondencesRefined;
    char poseFileIdx[100]; char poseRefinedFileIdx[100];
    {
        com.createDir("log");

        //open files for logging
        logFile.open("./log/logPoses.txt", std::ios::out);
        logMatrix.open("./log/logMatrix.txt", std::ios::out);
    }

    // handle the input arguments
    if (argc !=3)
    {
        cout << "running the simulation for " << LENGTHFRAME << " frames" << endl;
        cerr << "wrong number of input arguments..." << endl;
        cerr << "using default parameter instead..." << endl;

        startFrame = MINFRAMEIDX;
        endFrame   = MAXFRAMEIDX;
     }
    else
    {
        startFrame = atoi(argv[1]);
        endFrame   = atoi(argv[2]);
        cout << "running the simulation for " << endFrame - startFrame << " frames" << endl;
    }

    // load the point cloud, create kd-tree, and report the cloud dimension
    io::loadPCDFile(cloudPath, *cloud);
    kdtree.setInputCloud(cloud);
    cout << "loaded cloud with " << cloud->width * cloud->height << " points ("
         << getFieldsList (*cloud) << ")" << endl;

    // prepare the 2D, 3D and descriptor correspondences from files and initialise the lookuptable
    com.prepareMap(map2Dto3D, mapDesc, ref(_tunnel2D), ref(_tunnel3D), ref(_tunnelDescriptor));
    com.updatelut(_tunnel3D, _tunnelDescriptor, ref(_3dToDescriptorTable));

    // init all objects and vars for the main sequences, in order of definition
    int frameIndex = startFrame;
    int frameCount = 0;
    char currImgPath[100];
    vector<Frame> windowedFrame;

    Frame current;
    Frame prev;


    // start the positioning sequences
    while (frameIndex < endFrame)
    {
        cout << "\nprocessing frame-" << frameCount << "..." << endl;

        // log
        {
            sprintf(poseFileIdx, "./log/%d-poseFile.txt", frameIndex);
            sprintf(poseRefinedFileIdx, "./log/%d-poseRefined.txt", frameIndex);

            correspondences.open(poseFileIdx, std::ios::out);
            correspondencesRefined.open(poseRefinedFileIdx, std::ios::out);
        }


        // create local object of frame and automatically cleared every new iteration
        vector<int> matchesIndex3D;
        vector<int> matchesIndex2D;

        // load the image into the current frame
        if (GoPro)
            sprintf(currImgPath, "%s%04d.png", imgPath2.c_str(), frameIndex);               // this one is for the GOPRO
        else
            sprintf(currImgPath, "%simg_%05d.png", imgPath.c_str(), frameIndex);

        // init current Frame
        current = Frame();
        current.frameIdx = frameCount;
        current.image    = imread(currImgPath);

        // preprocess the image to remove visible outliers, e.g. dashboard
        Mat mask = Mat::zeros(current.image.size(), CV_8U);
        Mat roi (mask, Rect(current.image.cols*1/20, current.image.rows*1/20, current.image.cols*18/20, current.image.rows*17/20));
        roi = Scalar(255, 255, 255);


        // windowedFrame is a variable vector that contains n most recent Frames. It is used for tracking the most recent features and
        // also used for Bundle Adjustment (TODO)

        Mat lutDesc;
        // match the current frame descriptor with every frames in the window
        if (windowedFrame.size() != 0)
        {
            // detect features from current frame with provided region of interest mask
//            fdet.siftDetector(current.image, current.keypoints, mask);
            fdet.fastDetector(current.image, current.keypoints, mask);

            // extract feature descriptors from current frame
            fdet.siftExtraction(current.image, current.keypoints, current.descriptors);


            for (int i=windowedFrame.size()-1; i>=0; i--)
            {
                // if not empty, then correspondences has to be checked for every frames in the window
                lutDesc = com.getdescriptor(windowedFrame[i]._3dToDescriptor);

                // match descriptor
                fdet.bfMatcher(lutDesc, current.descriptors, current.matches);

                // perform lowe's ratio test, last parameter is for cout verbose (true/false)
                fdet.ratioTestRansac(current.matches, ref(windowedFrame[i]), ref(current), true);
            }


            // draw current frame's keypoints and matched landmarks
            {
                vector <Point2d> currKpts;

                for (int i=0; i<current.keypoints.size(); i++)
                    currKpts.push_back(Point2d(current.keypoints[i].pt.x, current.keypoints[i].pt.y));

                drawFeat(current.image, currKpts, current.matchedImagePoints, frameIndex);
            }

        }
        // if window empty --> if it's the first frame
        else
        {
            // current
            vector <Point3d> worldPoints;
            // set the image points
            vector <Point2d> imagePoints;


            //-- <! MOCKUP
            // this part is a mockup part which assuming we have 4 matches between image features and lookuptable
            {
                worldPoints.push_back(Point3d(143436.267, 6394357.480, 34.037)); //
                worldPoints.push_back(Point3d(143427.090, 6394362.320, 37.439)); //
                worldPoints.push_back(Point3d(143427.055, 6394361.512, 33.561)); //
                worldPoints.push_back(Point3d(143427.073, 6394362.323, 35.505)); //

                // 2D points for Volvo camera
                if (!GoPro)
                {
                    imagePoints.push_back(Point2d(887.8, 453.1)); //
                    imagePoints.push_back(Point2d(211.3, 268.4)); //
                    imagePoints.push_back(Point2d(211.2, 510.4)); //
                    imagePoints.push_back(Point2d(206.1, 401.5)); //
                }
                else
                // 2D points for GoPro camera
                {
                    imagePoints.push_back(Point2d(1365.970, 722.017));       // orange road sign
                    imagePoints.push_back(Point2d(485.9260, 489.672));       // left top
                    imagePoints.push_back(Point2d(486.0570, 805.360));       // left bottom
                    imagePoints.push_back(Point2d(483.0000, 660.000));       // left middle
                }
            }
            //-- >! MOCKUP

            // update the current frame's 2D-3D information
            current.matchedWorldPoints.insert(end(current.matchedWorldPoints), begin(worldPoints), end(worldPoints));
            current.matchedImagePoints.insert(end(current.matchedImagePoints), begin(imagePoints), end(imagePoints));

            // detect features from current frame with provided region of interest mask
//            fdet.siftDetector(current.image, current.keypoints, mask);
            fdet.fastDetector(current.image, current.keypoints, mask);

            // extract feature descriptors from current frame
            fdet.siftExtraction(current.image, current.keypoints, current.descriptors);
//
//
//            // if empty, the correspondences are obtained from the lookuptable
//            lutDesc = com.getdescriptor(_3dToDescriptorTable);
//
//            // clean LUT
//            _3dToDescriptorTable.clear();
//
//            // matcher
//            fdet.bfMatcher(lutDesc, current.descriptors, current.matches);
//
//            // perform David Lowe's ratio test. it gives 3D/2D indices to use in the next step
//            fdet.ratioTest(current.matches, ref(matchesIndex3D), ref(matchesIndex2D));
//
//            // retrieve the 3D from the lookup table, 2D from current frame's keypoints
//            if (matchesIndex2D.size() != matchesIndex3D.size())
//                cerr << "wrong size of 2D/3D correspondences" << endl;
//            else
//            {
//                for (int i=0; i<matchesIndex2D.size(); i++)
//                {
//                    current.matchedWorldPoints.push_back(Point3d(_3dToDescriptorTable[matchesIndex3D[i]].first.x,
//                                                                 _3dToDescriptorTable[matchesIndex3D[i]].first.y,
//                                                                 _3dToDescriptorTable[matchesIndex3D[i]].first.z));
//                    current.matchedImagePoints.push_back(Point2d(current.keypoints[matchesIndex2D[i]].pt.x,
//                                                                 current.keypoints[matchesIndex2D[i]].pt.y));
//                }
//            }
//
//            // use SIFT only for getting the match for the first LUT, then we use FAST-SIFT
//            current.keypoints.clear(); current.descriptors.release();
//            fdet.fastDetector(current.image, current.keypoints, mask);
//            fdet.siftExtraction(current.image, current.keypoints, current.descriptors);
        }

        // once we got the 3D and 2D correspondences and bigger than min correspondences, compute the camera pose
        if (current.matchedWorldPoints.size() < MINCORRESPONDENCES)
        {
            cerr << "not enough correspondences for PnP" << endl;
            exit(EXIT_FAILURE);
        }
        else
        {
            cout << "  found " << current.matchedWorldPoints.size() << " 3D points from lookup table window" << endl;

            // reinit solver
            solver = PnPSolver();
            solver.setPnPParam (PNPITERATION, PNPPIXELERROR, PNPCONFIDENCE);

            solver.setImagePoints(current.matchedImagePoints);
            solver.setWorldPoints(current.matchedWorldPoints);
            cout << "  result after matching...";
            solver.run(1);
        }

        // find the rest image points 3D representation using backprojections
        // prepare the matrices
        current.updateCameraParameters (cal.getCameraMatrix(),
                                        cal.getDistortionCoeffs(),
                                        solver.getRotationMatrix(),
                                        solver.getRotationVector(),
                                        solver.getTranslationVector(),
                                        solver.getCameraPose());

        // call the multithreaded backprojection wrapper
        com.threading(NUMTHREADS,
                      current.cameraPose,
                      current.K,
                      current.keypoints,
                      current.descriptors,
                      std::ref(cloud),
                      std::ref(kdtree),
                      std::ref(current._3dToDescriptor),                        // pair of 3d reprojected world points & descriptors (it's not LUT)
                      std::ref(current.reprojectedWorldPoints),                 // 3d reprojected world points
                      std::ref(current.reprojectedImagePoints),                 // 2d reprojected image points
                      std::ref(current.reprojectedIndices));                    // vector that contains reprojected points' indices

        cout << "  successfully reprojected " << current.reprojectedWorldPoints.size() << " points" << endl;



        // put the current frame into log
        {
            // save the current frame retrieved pair 2D and 3D
            for (int i=0; i<current.matchedImagePoints.size(); i++)
            {
                correspondences << std::fixed << setprecision(4)
                                << current.matchedWorldPoints[i].x << ", "
                                << current.matchedWorldPoints[i].y << ", "
                                << current.matchedWorldPoints[i].z << ", " << std::flush;

                correspondences << std::fixed << setprecision(4)
                                << current.matchedImagePoints[i].x << ", "
                                << current.matchedImagePoints[i].y << "\n" << std::flush;
            }

            // save the initial camera position
            logFile << std::fixed << setprecision(10)
                    << current.cameraPose.at<double>(0,3) << ", "
                    << current.cameraPose.at<double>(1,3) << ", "
                    << current.cameraPose.at<double>(2,3) << "\n" << std::flush;

            // save the initial Rotation and Translation matrices from PnP solver
            logMatrix << std::fixed << setprecision(10)
                      << current.R.at<double>(0,0) << ", " << current.R.at<double>(0,1) << ", " << current.R.at<double>(0,2) << ", "
                      << current.R.at<double>(1,0) << ", " << current.R.at<double>(1,1) << ", " << current.R.at<double>(1,2) << ", "
                      << current.R.at<double>(2,0) << ", " << current.R.at<double>(2,1) << ", " << current.R.at<double>(2,2) << ", "
                      << current.t.at<double>(0)   << ", " << current.t.at<double>(1)   << ", " << current.t.at<double>(2)   << "\n" << std::flush;


            for (int i=0; i<current.reprojectedWorldPoints.size(); i++)
            {
                correspondencesRefined << std::fixed << setprecision(4)
                                       << current.reprojectedWorldPoints[i].x << ", "
                                       << current.reprojectedWorldPoints[i].y << ", "
                                       << current.reprojectedWorldPoints[i].z << ", " << std::flush;

                correspondencesRefined << std::fixed << setprecision(4)
                                       << current.reprojectedImagePoints[i].x << ", "
                                       << current.reprojectedImagePoints[i].y << "\n" << std::flush;
            }
        }


        // update lookuptable:
        // the initial lookuptable has only small size of 3D-to-descriptor correspondences that needs to be updated
        // with most recent 3D-to-descriptors found using the multithreaded backprojection. it means that the newest
        // pairs will be inserted after the previous frame and the oldest frame in the window will be erased

        // !=old _3dToDescriptorTable.insert(end(_3dToDescriptorTable), begin(current._3dToDescriptor), end(current._3dToDescriptor));
        _3dToDescriptorTable.clear();
        _3dToDescriptorTable.insert(end(_3dToDescriptorTable), begin(current._3dToDescriptor), end(current._3dToDescriptor));
        cout << "  lookuptable has " << _3dToDescriptorTable.size() << " 3D-Descriptor pairs (before clearing)" << endl;

        // push frame into the window
        windowedFrame.push_back(current);

        // execute the bundle adjustment for both motion (R|t) and the structures (worldPoints)
        // after the execution, all frames's motion and worldPoints within sliding window will be optimized
        //bundle.run(ref(windowedFrame));



        if (windowedFrame.size() > WINDOWSIZE)
        {
            // clear the first index elements from lookup table
            // != old _3dToDescriptorTable.erase(_3dToDescriptorTable.begin(), _3dToDescriptorTable.begin() + windowedFrame[0].reprojectedWorldPoints.size());

            // clear the first frame inside window if the window is full
            windowedFrame.erase(windowedFrame.begin());
        }
        cout << "  lookuptable has " << _3dToDescriptorTable.size() << " 3D-Descriptor pairs (after clearing)" << endl;


        // now current becomes previous frame
        // TODO: still don't know how to use previous frame, verification maybe?
        prev = Frame();
        prev = current;



        // end of sequences, go to the next frameIndex
        frameIndex++;
        frameCount++;

        // log
        {
           correspondences.close();
           correspondencesRefined.close();
        }


        // wait for the user keyboard interrupt
        waitKey(1);



    }   // end the main loop


    // log
    {
        logFile.close();
        logMatrix.close();
    }


    // end of the main caller
    return 0;
}




// helper function definitions
void drawFeat(Mat img, vector<Point2d>& points1, vector<Point2d>& points2, int fidx)
{
    char imgName[100];

    Mat windowOutput;
    img.copyTo(windowOutput);

    namedWindow("Output Features", WINDOW_NORMAL);

    for (int i=0; i<points1.size(); i++)
    {
        //cv::line(windowOutput, points1[i], points2[i], CV_RGB(243, 225, 63));
        //cv::line(windowOutput, points1[i], points2[i], CV_RGB(82, 192, 249));
        cv::circle(windowOutput, points1[i], 1, CV_RGB(82, 192, 249));
    }

    for (int i=0; i<points2.size(); i++)
    {
        cv::circle(windowOutput, points2[i], 4, CV_RGB(243, 225, 63));
    }

    imshow("Output Features", windowOutput);
    sprintf(imgName, "%04d.png", fidx);
    imwrite(imgName, windowOutput);
}
