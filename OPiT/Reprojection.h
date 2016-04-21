#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Reprojection
{
public:
	Reprojection();

	cv::Mat foo(cv::Mat,cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat);

	static std::vector<double> backproject(cv::Mat T, cv::Mat K, cv::Point2d imagepoint, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


private:

};