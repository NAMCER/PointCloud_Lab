#pragma once
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;

///voxel downsampling
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;
	downSampled.setInputCloud(cloud_in);
	downSampled.setLeafSize(0.5f, 0.5f, 0.5f);
	downSampled.filter(*cloud_out);
	std::cout << "original point size:" << cloud_in->size() << std::endl;
	std::cout << "downSample point size:" << cloud_out->size() << std::endl;
	return cloud_out;
}

///normal vector
pcl::PointCloud<pcl::Normal>::Ptr calculaNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estim_normal;
	estim_normal.setInputCloud(cloud_in);
	estim_normal.setSearchMethod(tree);
	estim_normal.setKSearch(10);
	estim_normal.compute(*point_normal);

	return point_normal;
}

///FPFH
pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfphFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normal_in,pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(8);
	est_fpfh.setInputCloud(cloud_in);
	est_fpfh.setInputNormals(normal_in);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;
}

///Alignment

Eigen::Matrix4d TransMat(
	pcl::PointCloud<pcl::PointXYZ>::Ptr source,
	pcl::PointCloud<pcl::PointXYZ>::Ptr target,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh)
{
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
	sacia.setInputSource(source);
	sacia.setSourceFeatures(source_fpfh);
	sacia.setInputTarget(target);
	sacia.setTargetFeatures(target_fpfh);

	//??
	pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
	sacia.align(*align);

	Eigen::Matrix4d T(4, 4);
	T = sacia.getFinalTransformation().cast<double>();
	return T;
}