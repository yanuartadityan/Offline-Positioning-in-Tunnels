
#include <opencv2/features2d.hpp>
class Triangulation
{
public:
	Triangulation();
	Triangulation(cv::Mat CM);
	
	static cv::Mat LinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1);
	static cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1);
	static cv::Mat ocvTriangulation(cv::Mat cPose1, cv::Mat cPose2, std::vector<cv::Point2f> imgPoints1, std::vector<cv::Point2f> imgPoints2);

private:
	cv::Mat cameraMatrix;
};
