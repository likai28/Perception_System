// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Kai Li

#ifndef CLOUD_VISUALIZER_H
#define CLOUD_VISUALIZER_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vtkAutoInit.h>


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void cloudShow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &clouds);

#endif