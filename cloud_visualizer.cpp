// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Kai Li

#include "cloud_visualizer.h"

VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(0.001);
	viewer->initCameraParameters();
	return (viewer);
}

void cloudShow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clouds)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = rgbVis(clouds);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}