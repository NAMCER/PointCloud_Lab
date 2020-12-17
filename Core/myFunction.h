#pragma once
#include "global.h"



class cRandom
{
public:
	cRandom(int x, double y) :seed(x), random(y) {};
	cRandom() :seed(0), random(0) {};
	int seed;
	double random;
};

cRandom my_random(int z)
{
	const int m = pow(2, 31) - 1;
	const int a = 16807;
	const int q = 127773;
	const int r = 2836;

	int temp = a * (z % q) - r * (z / q);

	if (temp < 0)
	{
		temp = m + temp;
	}

	//z is the seed number
	z = temp;
	double t = z * 1.0 / m;
	cRandom cr;
	cr.random = t;
	cr.seed = z;
	return cr;
}

///IO File///
namespace pclfileIOStream
{
	///Read point cloud data
	bool readCloudfile(boost::filesystem::path readPath, pcl::PointCloud<pcl::PointXYZ>& outcloud)
	{
		if (readPath.empty())
		{
			PCL_ERROR("File patch error!!");
			return false;
		}
		if (readPath.extension() == ".PCD" || readPath.extension() == ".pcd")
		{
			pcl::io::loadPCDFile<pcl::PointXYZ>(readPath.string(), outcloud);
		}
		else
			pcl::io::loadPLYFile<pcl::PointXYZ>(readPath.string(), outcloud);

		return true;
	}

}


///make point clout data for testing
namespace baseMakedata
{
	bool makePointCloud(std::string& outPath)
	{
		Eigen::Matrix<float, 3, 1> sphereCenter(2.5, 2.5, 2.5);
		float radius = 10.0;
		const int num = pow(10, 5);
		int z1 = 20;
		int z2 = 112;
		cRandom angleV(z1, 0.0);
		cRandom angleH(z2, 0.0);

		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.width = num;
		cloud.height = 1;
		cloud.points.resize(cloud.width * cloud.height);

		for (size_t i = 0; i < num; ++i)
		{
			angleV = my_random(angleH.seed);
			angleH = my_random(angleV.seed);
			cloud.points[i].x = radius * sin(M_PI * angleV.random) * cos(2 * M_PI * angleH.random) + sphereCenter[0];
			cloud.points[i].y = radius * sin(M_PI * angleV.random) * sin(2 * M_PI * angleH.random) + sphereCenter[1];
			cloud.points[i].z = radius * cos(M_PI * angleV.random) + sphereCenter[2];
		}

		pcl::io::savePCDFile(outPath, cloud);
		std::cout << "make point cloud file, size of :" << cloud.size() << std::endl;
		//pcl::io::savePLYFile(outPath, cloud);
		return true;

	}

}

namespace baseFitting
{
	bool calculateSphereCoef(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		if (cloud->size()== 0)
			return false;

		//需要判断所有点的有效性
		//尚未实现
		
		Eigen::Matrix4f A;
		A.setZero();
		Eigen::Vector4f b(0, 0, 0, 0);
		Eigen::Vector4f temp(0, 0, 0, -1);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			Eigen::Vector3f pointTem = cloud->points[i].getArray3fMap();
			temp.block<3, 1>(0, 0) = pointTem.cast<float>();
			A += temp * temp.transpose();
			b += temp * (pointTem.transpose()*pointTem);
		}

		//solver
		Eigen::Vector4f coef = A.colPivHouseholderQr().solve(b);

		std::cout << "LS Coef :\n " << coef << std::endl;
		std::cout << "================ " << std::endl;
		std::cout << "a: " << coef[0] / 2.0 << std::endl;
		std::cout << "b: " << coef[1] / 2.0 << std::endl;
		std::cout << "c: " << coef[2] / 2.0 << std::endl;
		std::cout << "Radius: " << sqrt(coef[0]/2* coef[0]/2+ coef[1]/2* coef[1] / 2 + coef[2] / 2 * coef[2] / 2 - coef[3]) << std::endl;


	}

}


