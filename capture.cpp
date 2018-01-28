// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han & Kai Li

#include <opencv2/opencv.hpp>

#include <librealsense/rs.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

#include <ctime>
#include <iostream>

#include "calibration_parser.h"
#include "config.h"
#include "visual_modules.h"
#include "thermal_modules.h"
#include "cloud_visualizer.h"
#include "utilities.h"


int main(int argc, char* argv[])
{
	// Parse variables from command line
	std::string configFile(argv[1]);								// Config file
	std::string outputPath(argv[2]);								// Output path to save captured data
	std::string tempPath(argv[3]);									// Temporary path to save captured images


																	// Load config from file
	std::cout << "------------------Loading Configuration------------------" << std::endl;
	Config config;													// Config parameters
	config.init(configFile);


	// Initialize variables
	std::time_t time = std::time(nullptr);
	std::string timeStamp(std::to_string(static_cast<int>(time)));	// Current timestamp, which is used as the name of capture

	std::vector<cv::Mat> backgroundImgs;							// Background image for motion detection and image segmentation
	cv::Mat prevFrame;												// Previous frame for still image detection

	int mode = MOTION_DETECTION_MODE;								// Current mode of the workflow
	int numStill = 0;												// Numer of frames that are considered still when in capture mode

	std::vector<rs::device*> rsCameras;								// RealSense camera devices;
	rs::context rsContext;											// RealSense device context

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;		// Point clouds reconstructed from RS cameras
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(
		new pcl::PointCloud<pcl::PointXYZRGB>);						// Final output point cloud

	std::vector<std::vector<cv::KeyPoint>> pts2D;					// 2D positions of finger markers
	std::vector<Eigen::Vector3d> pts3D;								// 3D positions of touch points
	std::vector<int> success;										// Mark whether each finger is successfully captured


	// Initialize RealSense cameras
	std::cout << "------------------Initializing Cameras------------------" << std::endl;
	std::vector<std::string> camNames;								// Camera names
	rs::log_to_console(rs::log_severity::fatal);
	printRSContextInfo(&rsContext);
	rsCameras.resize(rsContext.get_device_count());
	for (int i = 0; i < rsCameras.size(); i++) {
		rsCameras[i] = rsContext.get_device(i);
		camNames.push_back(rsCameras[i]->get_serial());
	}


	// Load calibration parameters from file
	std::cout << "------------------Loading Calibration Parameters------------------" << std::endl;
	CalibrationParser colorCalibrationParser;						// Calibration parser
	CalibrationParser depthCalibrationParser;						// Calibration parser
	std::vector<Eigen::Matrix3d> depthIntrinsics;					// Depth camera intrinsics
	std::vector<Eigen::Matrix3d> colorIntrinsics;					// Depth camera intrinsics
	std::vector<Eigen::MatrixXd> depthToDepthExtrinsics;			// RGB camera extrinsics
	std::vector<Eigen::MatrixXd> colorToDepthExtrinsics;			// RGB camera extrinsics
	std::vector<Eigen::Matrix4d> refinedTransformation;				// Refined trainsformation after registration

	colorCalibrationParser.loadCalibration(config.colorCalibrationFile_);
	colorCalibrationParser.matchCameras(camNames);
	colorIntrinsics = colorCalibrationParser.getIntrinsics();
	colorToDepthExtrinsics = colorCalibrationParser.getExtrinsics();

	depthCalibrationParser.loadCalibration(config.depthCalibrationFile_);
	depthCalibrationParser.matchCameras(camNames);
	depthIntrinsics = depthCalibrationParser.getIntrinsics();
	depthToDepthExtrinsics = depthCalibrationParser.getExtrinsics();


	// Create temporary and output directories
	std::cout << "------------------Creating Directories------------------" << std::endl;
	std::string mkdirCmd = "mkdir " + tempPath;
	system(mkdirCmd.c_str());
	mkdirCmd = "mkdir " + outputPath;
	system(mkdirCmd.c_str());

	for (int i = 0; i < rsCameras.size(); i++) {
		std::string cmd = "mkdir " + tempPath + "cam" + std::to_string(i);
		system(cmd.c_str());
	}


	//char command = 'a';
	/*while (command != 'n' && command != 'N') {*/

	// Create output directory
	mkdirCmd = "mkdir" + outputPath + timeStamp;
	system(mkdirCmd.c_str());


		// Start cameras
	std::cout << "------------------Starting Cameras------------------" << std::endl;
	for (int i = 0; i < rsCameras.size(); i++) {
		initRSCamera(rsCameras[i], true);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME * 5));


	// Initialize background depth images
	std::cout << "------------------Initializing Depth Background------------------" << std::endl;
	std::cout << "Please wait ... ";
	initMultiBackgroundImg(
		rsCameras,
		backgroundImgs
	);
	std::cout << "Done." << std::endl;


	// Start surveillance
	std::cout << "------------------Starting Surveillance Mode------------------" << std::endl;
	executeMode(
		backgroundImgs,
		tempPath,
		config.waitTimeSec_,
		rsCameras,
		mode,
		numStill
	);
	std::this_thread::sleep_for(std::chrono::milliseconds(config.waitTimeSec_ * 1000));

	//// Perform 3D reconstruction
	std::cout << "------------------Reconstructing Point Cloud------------------" << std::endl;
	reconstructPointCloud(
		backgroundImgs,
		colorIntrinsics,
		depthIntrinsics,
		colorToDepthExtrinsics,
		depthToDepthExtrinsics,
		rsCameras,
		clouds,
		timeStamp
	);
	std::cout<< "------------------Object reconstruction is done, please start preheating process------------------" << std::endl;
	//// Register point clouds from different views
	//std::cout << "------------------Registering Point Cloud------------------" << std::endl;
	//registerPointCloud(
	//	depthToDepthExtrinsics,
	//	outputPath,
	//	tempPath,
	//	timeStamp,
	//	clouds,
	//	success,
	//	pts3D,
	//	outputCloud,
	//	refinedTransformation
	//);

	// Start the thermal modules
	std::cout << "------------------Starting the thermal modules------------------" << std::endl;

	// Define the 2D touching points
	std::vector<cv::Mat> centers;

	std::cout << "------------------Please enter Y or y after you finish preheat process and touch on the object------------------" << std::endl;
	char thermalCommand = 'a';
	while (true)
	{
		std::cin >> thermalCommand;
		if (thermalCommand == 'Y' || thermalCommand == 'y')
		{
			break;
		}
	}
	std::cout << "------------------Starting thermal Segmentation process------------------" << std::endl;

	// Get the center points from four views of thermal cameras
	centers = thermalSegmentation();

	// Reproject the 2D thermal points back to 3D point clouds
	thermalReproject(rsCameras,clouds,centers);

	//Stitich point clouds from multiple views together
	std::vector<Eigen::Matrix4d> transformations(depthToDepthExtrinsics.size());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < depthToDepthExtrinsics.size(); i++) {
		transformations[i].block(0, 0, 3, 3) = depthToDepthExtrinsics[i].block(0, 0, 3, 3);
		transformations[i].block(0, 3, 3, 1) = depthToDepthExtrinsics[i].block(0, 3, 3, 1) * MILLI_TO_METER_SCALE;
		transformations[i](3, 0) = 0.0;
		transformations[i](3, 1) = 0.0;
		transformations[i](3, 2) = 0.0;
		transformations[i](3, 3) = 1.0;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*clouds[i], *tmp, transformations[i]);
		*output += *tmp;
	}
	pcl::io::savePCDFile(".\\output\\together.pcd", *output);

	std::cout << "------------------Job Done------------------" << std::endl;
	std::cout << "Capture finished. All files saved to " + outputPath + timeStamp << std::endl << std::endl;

	// Visualize the 3D point cloud
	cloudShow(output);

	// Release temporary memory and update timestamp
	time = std::time(nullptr);
	timeStamp = std::string(std::to_string(static_cast<int>(time)));
	pts2D.clear();
	pts3D.clear();
	backgroundImgs.clear();
	prevFrame = cv::Mat();
	numStill = 0;
	for (int i = 0; i < rsCameras.size(); i++) {
		rsCameras[i]->stop();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME * 5));
		//do {
		//	std::cout << "Start another capture? (y/n)" << std::endl;
		//	std::cin >> command;
		//} while (command != 'y' && command != 'Y' && command != 'n' && command != 'N');
	//}

	return 0;
}

