﻿#pragma once
#include "global.h"
#include "FPFH_Align.h"

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
namespace fileIOStream
{
	///获取文件夹下的所有文件，返回个文件的路径
	void getFile(std::string path ,std::vector<std::string> &files)
	{
		intptr_t hFile = 0;
		struct _finddata_t fileinfo;
		std::string p;
		if((hFile =_findfirst(p.assign(path).append("\\*").c_str(),&fileinfo))!=-1)
			{
				do 
				{
					if ((fileinfo.attrib & _A_SUBDIR))
					{
						if (strcmp(fileinfo.name,".") != 0 && strcmp(fileinfo.name,"..") != 0)
						{
							getFile(p.assign(path).append("\\").append(fileinfo.name), files);
						}
					}
					else
					{
						files.push_back(path + "\\" + fileinfo.name);
					}
				} while (_findnext(hFile,&fileinfo) == 0);
				_findclose(hFile);
			}
	}

	///获取文件夹下的所有文件，返回个文件的路径与文件名
	void getFile2(std::string path, std::vector<std::string> &files,std::vector<string> &ownname)
	{
		intptr_t hFile = 0;  //long type for windows 7
		struct _finddata_t fileinfo;
		std::string p;
		if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				if ((fileinfo.attrib & _A_SUBDIR))
				{
					if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					{
						getFile2(p.assign(path).append("\\").append(fileinfo.name), files,ownname);
					}
				}
				else
				{
					files.push_back(path + "\\" + fileinfo.name);
					ownname.push_back(fileinfo.name);
				}
			} while (_findnext(hFile, &fileinfo) == 0);
			_findclose(hFile);
		}
	}
	
	///Read point cloud data
	bool readCloudfile(boost::filesystem::path readPath, pcl::PointCloud<pcl::PointXYZ>& outcloud)
	{
		if (readPath.empty())
		{
			PCL_ERROR("File patch error!!");
			return SYSTEM_FAIL;
		}

		if (readPath.extension() == ".PCD" || readPath.extension() == ".pcd")
		{
			pcl::io::loadPCDFile<pcl::PointXYZ>(readPath.string(), outcloud);
		}
		else
			pcl::io::loadPLYFile<pcl::PointXYZ>(readPath.string(), outcloud);

		return SYSTEM_SUCESS;
	}
	///Save point cloud data Not Organized
	bool saveCloudfile(std::string outPath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		if (!(outPath.empty()) || !(cloud->empty()))
		{
			pcl::io::savePCDFileASCII(outPath, *cloud);
			return SYSTEM_SUCESS;
		}
		else
			return SYSTEM_FAIL;
	}
}

///Mack PointCloud File///
namespace makePointData
{
	///make sphere
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

		return SYSTEM_SUCESS;
	}

}

///Sampling
namespace makeSampling
{
	///Down sampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr DownSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDst(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> filter;
		filter.setInputCloud(cloud);
		// 
		filter.setLeafSize(leafSize, leafSize, leafSize);
		filter.filter(*cloudDst);

		return cloudDst;
	}

	///Uniform sampling
	pcl::PointCloud<pcl::PointXYZ>::Ptr UniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDst(new pcl::PointCloud<pcl::PointXYZ>);
		// Uniform sampling object.
		pcl::UniformSampling<pcl::PointXYZ> filter;
		filter.setInputCloud(cloud);
		filter.setRadiusSearch(0.01f);
		// We need an additional object to store the indices of surviving points.
		pcl::PointCloud<int> keypointIndices;


		//filter.compute(keypointIndices);
		//pcl::copyPointCloud(*cloud, keypointIndices.points, *cloudDst);
		filter.filter(*cloudDst);
		return cloudDst;
	}

	///Add sampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr UpSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDst(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
		filter.setInputCloud(cloud);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
		filter.setSearchMethod(kdtree);
		filter.setSearchRadius(0.03);
		filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
		filter.setUpsamplingRadius(0.03);
		filter.setUpsamplingStepSize(0.02);

		filter.process(*cloudDst);
		return cloudDst;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr samplingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc, Sampling_Type sType)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDst(new pcl::PointCloud<pcl::PointXYZ>);
		switch (sType)
		{
		case DOWN_SAMPLING:
			cloudDst = DownSampling(cloudSrc, 0.05);//0.01 =1cm
			break;
		case UNIFORM_SAMPLING:
			cloudDst = UniformSampling(cloudSrc);
			break;
		case UP_SAMPLING:
			cloudDst = UpSampling(cloudSrc);
			break;
		default:
			return 0;
			break;
		}
		return cloudDst;
	}

}

///All fitting function///
namespace allFitting
{
	bool fitingSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f& coef)
	{
		if (cloud->size() == 0)
			return false;

		Eigen::Vector4f b(0, 0, 0, 0);
		Eigen::Matrix4f A;
		A.setZero();
		Eigen::Vector4f tmp(0, 0, 0, -1);

		for (int i = 0; i < cloud->size(); i++)
		{
			Eigen::Vector3f pointTmp = cloud->points[i].getArray3fMap();

			tmp.block<3, 1>(0, 0) = pointTmp.cast<float>();
			A += tmp * tmp.transpose();
			b += tmp * (pointTmp.transpose() * pointTmp);
		}

		Eigen::Vector4f param = A.colPivHouseholderQr().solve(b);
		coef[0] = param[0] / 2; //a
		coef[1] = param[1] / 2; //b
		coef[2] = param[2] / 2; //c
		coef[3] = sqrt((param[0] / 2 * param[0] / 2 + param[1] / 2 * param[1] / 2 + param[2] / 2 * param[2] / 2) - param[3]); //R

		return SYSTEM_SUCESS;
	}

	///A = UsigmaV
	bool fitingPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f& coef)
	{
		Eigen::Vector3f Pcenter = Eigen::Vector3f::Zero();

		for (auto iter : cloud->points)
		{
			Pcenter[0] += iter.x;
			Pcenter[1] += iter.y;
			Pcenter[2] += iter.z;
		}
		Pcenter /= cloud->points.size();

		Eigen::MatrixXd A(cloud->points.size(), 3);
		for (int i = 0; i < cloud->points.size(); i++) {
			A(i, 0) = cloud->points[i].x - Pcenter[0];
			A(i, 1) = cloud->points[i].y - Pcenter[1];
			A(i, 2) = cloud->points[i].z - Pcenter[2];
		}

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
		const float a = svd.matrixV()(0, 2);
		const float b = svd.matrixV()(1, 2);
		const float c = svd.matrixV()(2, 2);

		const float d = -(a * Pcenter[0] + b * Pcenter[1] + c * Pcenter[2]);
		Eigen::Vector4d(a, b, c, d);
		coef = (Eigen::Vector4d(a, b, c, d)).cast<float>();

		return true;
	}

	///Eigen Vector 
	bool fitingPlane2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f& coef)
	{
		Eigen::Vector3f Pcenter = Eigen::Vector3f::Zero();

		for (auto iter : cloud->points)
		{
			Pcenter[0] += iter.x;
			Pcenter[1] += iter.y;
			Pcenter[2] += iter.z;
		}
		Pcenter /= cloud->points.size();
		//coverience Ax=0 => Atranspos *A x = 0 x=[a,b,c,-d]
		Eigen::Matrix4f A;
		A.setZero();
		Eigen::Vector4f tmp(0, 0, 0, 1);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			Eigen::Vector3f pointTmp(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
			tmp.block<3, 1>(0, 0) = (pointTmp - Pcenter).cast<float>();
			A += tmp * tmp.transpose();
		}
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver(A);
		Eigen::Vector4f param = solver.eigenvectors().col(0);

		const float a = param[0];
		const float b = param[1];
		const float c = param[2];
		const float d = -(param[0] * Pcenter[0] + param[1] * Pcenter[1] + param[2] * Pcenter[2]); // ax+by+cz-d=0
		Eigen::Vector4d(a, b, c, d);
		coef = (Eigen::Vector4d(a, b, c, d)).cast<float>();
		return 0;
	}

	///PCL RANSAC
	bool pclfitingPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f& coef)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.02);
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		const float a = coefficients->values[0];
		const float b = coefficients->values[1];
		const float c = coefficients->values[2];
		const float d = coefficients->values[3];

		Eigen::Vector4d(a, b, c, d);
		coef = (Eigen::Vector4d(a, b, c, d)).cast<float>();

		return SYSTEM_SUCESS;
	}

}

///Calculate 3D feature function
namespace Calculate3Dfeaturs
{
	///pcl(KdTree) Normal vector 
	bool pclCalcuNormalVec(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nv;
		nv.setInputCloud(cloud);
		//set search method
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr kdt(new pcl::search::KdTree<pcl::PointXYZ>);
		//nv.setSearchMethod(kdt);
		
		pcl::search::Octree<pcl::PointXYZ> octree(128.0f);
		octree.setInputCloud(cloud);
		//save output data
		//pcl::PointCloud<pcl::Normal>::Ptr locNormals(new pcl::PointCloud<pcl::Normal>);	
		nv.setRadiusSearch(0.4);
		nv.compute(*normals);

		return SYSTEM_SUCESS;
	}
}

///PCL Visualization 
namespace pclVisualization
{
	bool viewVisualiz(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		if (!(cloud->empty()) && !(normals->empty()))
		{
			viewer->setBackgroundColor(0.3, 0.3, 0.3);
			viewer->addText("Normal vector", 10, 10, "text");
			//setting color
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fildColor(cloud, 0, 0, 255);
			//add coordination
			viewer->addCoordinateSystem(0.1);
			viewer->addPointCloud(cloud, fildColor, "Point cloud");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point cloud");
			viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 1, "normals");
		}
		if (!(cloud->empty()) && (normals->empty()))
		{
			viewer->setBackgroundColor(0.3, 0.3, 0.3);
			viewer->addText(" Point Cloud Show Time", 10, 10, "text");
			//setting color
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fildColor(cloud, 0, 0, 255);
			//add coordination
			viewer->addCoordinateSystem(0.1);
			viewer->addPointCloud(cloud, fildColor, "Point cloud");
		}

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		return SYSTEM_SUCESS;
	}
}
///Filterling
namespace Filterling
{
	//outlier Remove
	bool pclOutlierRemovalfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud,NegativeORPositive flags)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(incloud);
		sor.setMeanK(50); //邻近点数个数
		sor.setStddevMulThresh(1.0);//? 阈值	
		switch (flags)
		{
		case NEGATIVE:
			sor.setNegative(NEGATIVE);
			break;
		case POSITIVE:
			sor.setNegative(POSITIVE);
			break;
		default:
			sor.setNegative(NEGATIVE);
			break;
		}
		sor.filter(*outcloud);

		return SYSTEM_SUCESS;
	}
}

///key point
namespace KeyPointExtract
{
	bool NARF_P(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		bool setUnseenToMaxRange = false;
		float angular_resolution = 0.5f;
		float support_size = 0.2f;

		angular_resolution = pcl::deg2rad(angular_resolution);
		//creat range image
		Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
		float noise_level = 0.0;
		float min_range = 0.0f;
		int border_size = 1;
		

		boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;
		range_image.createFromPointCloud(*cloud, angular_resolution, pcl::deg2rad(360.0f),
			pcl::deg2rad(180.0f),scene_sensor_pose,coordinate_frame,noise_level,min_range,border_size);

		if (setUnseenToMaxRange)
			range_image.setUnseenToMaxRange();
		pcl::visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(1, 0.5, 1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
		viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

		// --------------------------  
		// -----Show range image-----  
		// --------------------------  
		pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
		range_image_widget.showRangeImage(range_image);
		// -----Extract NARF keypoints----- 
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage(&range_image);
		narf_keypoint_detector.getParameters().support_size = support_size;

		pcl::PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute(keypoint_indices);
		std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

		// -------------------------------------  
		// -----Show keypoints in 3D viewer-----  
		// -------------------------------------  
		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
		keypoints.points.resize(keypoint_indices.points.size());
		for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
			keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
		viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
		
		while (!viewer.wasStopped())
		{
			//range_image_widget.spinOnce();  // process GUI events  
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
}

/// svd Rotation and translation
namespace MY_SVD
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
	void pose_estimation_3d3d(pcl::PointCloud<pcl::PointXYZ>::Ptr pts1
		, pcl::PointCloud<pcl::PointXYZ>::Ptr pts2
		, Eigen::Matrix3d &R
		, Eigen::Vector3d &t)
	{
		Eigen::Vector3f Pcenter1 = Eigen::Vector3f::Zero();
		int pts1_size = pts1->size();
		int pts2_size = pts2->size();
		for (auto iter : pts1->points)
		{
			Pcenter1[0] += iter.x;
			Pcenter1[1] += iter.y;
			Pcenter1[2] += iter.z;
		}
		Pcenter1 /= pts1->points.size();
		std::cout << "pts1 center :" << Pcenter1[0] << "," << Pcenter1[1] << "," << Pcenter1[2] << std::endl;

		std::vector<pcl::PointXYZ> q1(pts1_size);
		for (int i = 0; i<pts1_size; ++i)
		{
			q1[i].x =pts1->points[i].x - Pcenter1[0];
			q1[i].y = pts1->points[i].y - Pcenter1[1];
			q1[i].z = pts1->points[i].z - Pcenter1[2];
		}

		Eigen::Vector3f Pcenter2 = Eigen::Vector3f::Zero();
		for (auto iter:pts2->points)
		{
			Pcenter2[0] += iter.x;
			Pcenter2[1] += iter.y;
			Pcenter2[2] += iter.z;
		}
		Pcenter2 /= pts2->points.size();
		std::cout << "pts2 center :" << Pcenter2[0] << "," << Pcenter2[1] << "," << Pcenter2[2] << std::endl;
		
		std::vector<pcl::PointXYZ> q2(pts2_size);
		for (int i = 0; i < pts2_size; ++i)
		{
			q2[i].x = pts2->points[i].x - Pcenter2[0];
			q2[i].y = pts2->points[i].y - Pcenter2[1];
			q2[i].z = pts2->points[i].z - Pcenter2[2];
		}




	}
}

//test
namespace testFunc
{

	bool octreeTest(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Localcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		/*
		{

			// Generate pointcloud data
			Localcloud->width = 100000;
			Localcloud->height = 1;
			Localcloud->points.resize(Localcloud->width * Localcloud->height);
#pragma omp parallel for 
			for (int i = 0; i < Localcloud->points.size(); ++i)
			{
				Localcloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
				Localcloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
				Localcloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);

				Localcloud->points[i].r = 255;
				Localcloud->points[i].g = 255;
				Localcloud->points[i].b = 255;
			}

		}
		*/

		Localcloud->width = cloud->points.size();
		Localcloud->height = 1;
		Localcloud->points.resize(Localcloud->width * Localcloud->height);

#pragma omp parallel for 
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			Localcloud->points[i].x = cloud->points[i].x;
			Localcloud->points[i].y = cloud->points[i].y;
			Localcloud->points[i].z = cloud->points[i].z;

			Localcloud->points[i].r = 255;
			Localcloud->points[i].g = 255;
			Localcloud->points[i].b = 255;
		}

		float resolution = 0.1f;//设置octree体素分辨率
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);//建立octree对象
		octree.setInputCloud(Localcloud);//传入需要建立octree的点云指针
		//octree.defineBoundingBox();
		//octree.defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
		octree.addPointsFromInputCloud();  //构建Octree
		// create a virtual searchPoint
		//pcl::PointXYZ searchPoint(-0.221520, 0.121360, -1.195600);
		// create a virtual searchPoint

		pcl::PointXYZRGB searchPoint(-0.362300, 0.190920, -1.315000);
		// Neighbors within voxel search
		std::vector<int> pointIdxVec;

		if (octree.voxelSearch(searchPoint, pointIdxVec))
		{
			std::cout << "Neighbors within voxel search at (" << searchPoint.x
				<< " " << searchPoint.y
				<< " " << searchPoint.z << ")"
				<< std::endl;
#pragma omp parallel for
			for (int i = 0; i < pointIdxVec.size(); ++i)
			{
				Localcloud->points[pointIdxVec[i]].r = 255;
				Localcloud->points[pointIdxVec[i]].g = 0;
				Localcloud->points[pointIdxVec[i]].b = 0;
			}

		}
		// K nearest neighbor search
		searchPoint = pcl::PointXYZRGB(-0.314320, -0.504410, -1.514100);
		int K = 100;
		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquaredDistance;

		std::cout << "K nearest neighbor search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with K=" << K << std::endl;

		if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
#pragma omp parallel for 
			for (int i = 0; i < pointIdxNKNSearch.size(); ++i)
			{
				Localcloud->points[pointIdxNKNSearch[i]].r = 0;
				Localcloud->points[pointIdxNKNSearch[i]].g = 255;
				Localcloud->points[pointIdxNKNSearch[i]].b = 0;
			}
		}

		// Neighbors within radius search
		searchPoint = pcl::PointXYZRGB(0.289600, 0.141260, -1.231900);
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
		std::cout << "Neighbors within radius search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with radius=" << radius << std::endl;

		if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
#pragma omp parallel for
			for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				Localcloud->points[pointIdxRadiusSearch[i]].r = 0;
				Localcloud->points[pointIdxRadiusSearch[i]].g = 0;
				Localcloud->points[pointIdxRadiusSearch[i]].b = 255;
			}

		}

		pcl::visualization::CloudViewer viewer("cloud viewer");
		viewer.showCloud(Localcloud);

		system("pause");

		octree.deleteTree();

		return SYSTEM_SUCESS;

	}

}

namespace FPFHalign
{
	void runPFPHalign(std::string sourcePath, std::string targetPath)
	{
		clock_t start, end, time;
		start = clock();
		pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_voxel(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_voxel(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_align(new pcl::PointCloud<pcl::PointXYZ>);
		
		///Read point cloud
		fileIOStream::readCloudfile(sourcePath, *source);
		fileIOStream::readCloudfile(targetPath, *target);

		/*
		std:vector<int> index;
		pcl::removeNaNFromPointCloud(*source, *source, index);
		pcl::removeNaNFromPointCloud(*target, *target, index);
		*/
		//source_voxel = voxelGrid(source);
		//target_voxel = voxelGrid(target);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		pcl::PointCloud<pcl::Normal>::Ptr source_normal = calculaNormal(source/*source_voxel*/, tree);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh = pfphFeature(source/*source_voxel*/, source_normal, tree);

		pcl::PointCloud<pcl::Normal>::Ptr target_normal = calculaNormal(target/*target_voxel*/, tree);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh = pfphFeature(target/*target_voxel*/, target_normal, tree);
		
		end = clock();
		std::cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

		Eigen::Matrix4d T = TransMat(source/*source_voxel*/, target/*target_voxel*/, source_fpfh, target_fpfh);
		
		std::cout << "T:" << std::endl;
		std::cout << T << std::endl;

		Eigen::Matrix3d R = T.block<3, 3>(0, 0);
		Eigen::Quaterniond q = Eigen::Quaterniond(R);
		std::cout << "Quaternion = \n" << q.coeffs().transpose() << endl;

		Eigen::MatrixXd t_mat = T.topRightCorner(3, 1).transpose();
		Eigen::Vector3d t;
		t << t_mat(0), t_mat(1), t_mat(2);
		cout << "t = " << endl << t.transpose() << endl;

		ofstream outfile("Transform.txt");
		if (!outfile)
		{
			cerr << "open error!" << endl;
			exit(1);
		}
		outfile << "T:" << endl << T << endl;
		outfile << "q:" << endl << q.coeffs().transpose() << endl;
		outfile << "t:" << endl << t.transpose() << endl;
		outfile.close();

		pcl::transformPointCloud(*source, *source_align, T);
		pcl::io::savePCDFile("source_align.pcd", *source_align, false);

		///viz
		 //设置特征点对的关系
		pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
		boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);//FPFP的点对的对应关系
		crude_cor_est.setInputSource(source_fpfh);
		crude_cor_est.setInputTarget(target_fpfh);
		//crude_cor_est.determineCorrespondences(*cru_correspondences);
		crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
		cout << "crude size is:" << cru_correspondences->size() << endl;

		boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
		int v1;
		int v2;
		view->createViewPort(0, 0.0, 0.5, 1.0, v1);
		view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		view->setBackgroundColor(0, 0, 0, v1);
		view->setBackgroundColor(0.05, 0, 0, v2);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
		view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);
		view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
		//设置点的大小为2
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v1");

		view->addPointCloud(source_align, sources_cloud_color, "aligend_cloud_v2", v2);
		view->addPointCloud(target, target_cloud_color, "target_cloud_v2", v2);
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

		view->addCorrespondences<pcl::PointXYZ>(source, target, *cru_correspondences, "correspondence", v1);//添加显示对应点对
		//view->addCorrespondences<pcl::PointXYZ>(source_align,target,*cru_correspondences,"correspondence",v2);
		while (!view->wasStopped())
		{
			// view->spin();
			view->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}

	}

}
