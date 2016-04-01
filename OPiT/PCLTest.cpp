#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "PCLTest.h"

void PCLTest::foo()
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	// F i l l i n t h e c l o u d d a t a
	cloud.width = 500;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	for(size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	pcl::io::savePCDFileASCII("testpcd.pcd", cloud);
	
}