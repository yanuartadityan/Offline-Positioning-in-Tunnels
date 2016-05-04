class PCLCloudSearch
{
public:
	static std::vector<double> FindClosestPoint(double,double,double, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	static void VoxelizeCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);
private:
};
