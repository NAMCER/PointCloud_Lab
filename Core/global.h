#pragma once
#include <iostream>
#include<fstream>
#include <cstdlib>
#include <math.h>
#include <vector>
///PCL///
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

///VTK///
//#include <vtkAutoInit.h>
//#include <vtkPoints.h>
//#include <vtkSmartPointer.h>
//#include <vtkPolyData.h>
//#include <vtkVertexGlyphFilter.h>
//#include <vtkElevationFilter.h>
//#include <vtkPolyDataMapper.h>
//#include <vtkActor.h>
//#include <vtkRenderWindow.h>
//#include <vtkRenderWindowInteractor.h>
//#include <vtkRenderer.h>
//#include <vtkInteractorStyleTrackballCamera.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
///EIGEN///
#include <Eigen/core>
#include <Eigen/Dense>
///TBB///
//#include <tbb/tbb_thread.h>

///Boost///
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

/// enum
enum Result_State
{
	SYSTEM_FAIL = 0,
	SYSTEM_SUCESS = 1,
};

enum Sampling_Type
{
	DOWN_SAMPLING = 1,
	UNIFORM_SAMPLING = 2,
	UP_SAMPLING = 3,
};