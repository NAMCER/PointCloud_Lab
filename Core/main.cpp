
#include "myFunction.h"


int main(int argc, char** argv)
{
	std::string filePath = R"(../Data/out.pcd)";
	std::string outPath = R"(../Data/out.pcd)";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pclfileIOStream::readCloudfile(filePath, *cloud);
	baseFitting::calculateSphereCoef(cloud);

	//makePointCloud(outPath);

	std::cout << "Read Point size: " << cloud->size() << std::endl;

	system("pause");

	return (0);

}