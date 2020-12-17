#include "myFunction.h"

void test_func(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	if (0)///
	{
		Eigen::Vector4f sphereCoef;
		allFitting::fitingSphere(cloud, sphereCoef);
		std::cout << "Sphere Coef:\n " << sphereCoef << std::endl;
	}
	if (0)///
	{
		Eigen::Vector4f coef(0, 0, 0, 0);
		allFitting::fitingPlane(cloud, coef);
		std::cout << "fitingPlane  Coef:\n " << coef << std::endl;

		coef.setZero();
		allFitting::fitingPlane2(cloud, coef);
		std::cout << "fitingPlane2  Coef:\n " << coef << std::endl;

		coef.setZero();
		allFitting::pclfitingPlane(cloud, coef);
		std::cout << "pclfitingPlane  Coef:\n " << coef << std::endl;
	}
	if (0) ///
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		Calculate3Dfeaturs::pclCalcuNormalVec(cloud, normals);
		pclVisualization::viewVisualiz(cloud, normals);

	}
	if (1)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZ>);
		outCloud = makeSampling::samplingPoints(cloud, DOWN_SAMPLING);
		fileIOStream::saveCloudfile("../Data/cloud_down.pcd", outCloud);
	}

}

int main(int argc, char** argv)
{
	std::string filePath = R"(../Data/1.pcd)";
	std::string outPath = R"(../Data/out.pcd)";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	fileIOStream::readCloudfile(filePath, *cloud);
	std::cout << "Point size: " << cloud->size() << std::endl;

	test_func(cloud);

	system("pause");
	return (0);
}

