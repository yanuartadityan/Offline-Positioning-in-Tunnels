#include "Common.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <mutex>
#include <sys/stat.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace std::chrono;
using namespace cv;

// constructor
Common::Common()
{

}

// destructor()
Common::~Common()
{

}

// timer
void Common::startTimer()
{
    t1 = high_resolution_clock::now();
}

// timer
void Common::reportTimer()
{
    t2 = high_resolution_clock::now();
    auto duration1 = duration_cast<milliseconds> (t2-t1).count();
    cout << "  finish in " << duration1 << "ms (" << duration1/1000 << " sec)\n" << endl;
}

// log
void  Common::createDir(const string dirname)
{
    char logDirChar[100];
    strcpy(logDirChar, dirname.c_str());
    logDirChar[sizeof(logDirChar) - 1] = 0;

    mkdir(logDirChar, 0777);
}

// prepare map
void Common::prepareMap (char* mapCoordinateFile, char* mapKeypointsFile, vector<Point2d> *tunnel2Dx, vector<Point3d> *tunnel3Dx, Mat *tunnelDescriptor)
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
            tunnel2Dx->push_back(Point2f(a,b));
            tunnel3Dx->push_back(Point3f(x,y,z));
        }
    }
    mapFile.close();
    
    // load keypoints
    cv::FileStorage lstorage(mapKeypointsFile, cv::FileStorage::READ);
    lstorage["img"] >> *tunnelDescriptor;
    lstorage.release();
}
