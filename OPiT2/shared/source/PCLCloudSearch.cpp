#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include "PCLCloudSearch.h"


std::vector<double> PCLCloudSearch::FindClosestPoint(double x, double y, double z, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud(cloud);

	pcl::PointXYZ searchPoint;

	searchPoint.x = x;
	searchPoint.y = y;
	searchPoint.z = z;

	// K nearest neighbor search, we want only the nearest point.
	int K = 1;

	std::vector<double> bestPoint{0, 0, 0, 1000};

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	//std::cout << "K nearest neighbor search at (" << searchPoint.x
	//	<< " " << searchPoint.y
	//	<< " " << searchPoint.z
	//	<< ") with K=" << K << std::endl;

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
		{
			//std::cout	<< "    " << cloud->points[pointIdxNKNSearch[i]].x
			//			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			//			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			//			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
			//
			if (pointNKNSquaredDistance[i] < bestPoint[3])
			{
				bestPoint[0] = cloud->points[pointIdxNKNSearch[i]].x;
				bestPoint[1] = cloud->points[pointIdxNKNSearch[i]].y;
				bestPoint[2] = cloud->points[pointIdxNKNSearch[i]].z;
				bestPoint[3] = pointNKNSquaredDistance[i];
			}
		}
	}
	return bestPoint;
}

void PCLCloudSearch::VoxelizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered)
{
	std::cerr 	<< "PointCloud before filtering: " << cloud->width * cloud->height
       			<< " data points (" << pcl::getFieldsList (*cloud) << ")" << std::endl;

	// Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> voxelized;
	voxelized.setInputCloud (cloud);
	voxelized.setLeafSize (0.05f, 0.05f, 0.05f);
	voxelized.filter (*cloudFiltered);

	std::cerr 	<< "PointCloud after filtering: " << cloudFiltered->width * cloudFiltered->height
       			<< " data points (" << pcl::getFieldsList (*cloudFiltered) << ")" << std::endl;

    pcl::io::savePCDFile("cloud-voxelized.pcd", *cloudFiltered);
	std::cerr << "Saved " << cloudFiltered->points.size () << " data points to [voxelized.pcd]" << std::endl;
}
