#include <iostream>
#include <fstream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "PCLCloudSearch.h"

using namespace std;
using namespace pcl;

int main ()
{
    // load pcl
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloudFiltered(new PointCloud<PointXYZ>);
    io::loadPCDFile("cloud-big.pcd", *cloud);

    // downsample
    PCLCloudSearch::VoxelizeCloud (cloud, cloudFiltered);

    return 0;
}
