// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han

#ifndef MODULES_H
#define MODULES_H

#include <opencv2/opencv.hpp>

#include <librealsense/rs.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/convolution_3d.h>
#include <string>

#include "utilities.h"
#include "constants.h"
#include <omp.h>


void initMultiBackgroundImg(
	std::vector<rs::device*> rsCameras,
	std::vector<cv::Mat> &backgroundImg
);	// Initialize the background depth maps of multiple cameras


bool motionDetectionInteration(
	const cv::Mat &backgroundImg,
	std::vector<rs::device*> &rsCameras
);	// Detect motion in current frame


void captureIteration(
	const std::vector<cv::Mat> &backgroundImgs,
	const std::string tempPath,
	std::vector<rs::device*> &rsCameras,
	cv::Mat &prevFrame,
	int &numStill
);	// Detect still frames and capture 


void generatePointCloud(
	const Eigen::Matrix3d &colorIntrinsic,
	const Eigen::Matrix3d &depthIntrinsic,
	const Eigen::MatrixXd &colorToDepthExtrinsic,
	const cv::Mat &backgroundImg,
	const cv::Mat &depthImg,
	const cv::Mat &colorImg,
	rs::device *dev,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
);	// Generate point clouds for a single camera


void reconstructPointCloud(
	const std::vector<cv::Mat> &backgroundImgs,
	const std::vector<Eigen::Matrix3d> &colorIntrinsics,
	const std::vector<Eigen::Matrix3d> &depthIntrinsics,
	const std::vector<Eigen::MatrixXd> &colorToDepthExtrinsics,
	const std::vector<Eigen::MatrixXd> &depthToDepthExtrinsics,
	const std::vector<rs::device*> &rsCameras,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds,
	std::string timeStamp
);	// Generate point clouds for multiple cameras and perform registration


void registerPointCloud(
	const std::vector<Eigen::MatrixXd> &depthToDepthExtrinsics,
	const std::string outputPath,
	const std::string tempPath,
	std::string timeStamp,
	const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds,
	const std::vector<int> &success,
	std::vector<Eigen::Vector3d> &pts3D,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
	std::vector<Eigen::Matrix4d> &refinedTransformation
);


void executeMode(
	const std::vector<cv::Mat> &backgroundImgs,
	const std::string &tempPath,
	const int waitTimeSec,
	std::vector<rs::device*> &rsCameras,
	int mode,
	int &numStill
);	

#endif