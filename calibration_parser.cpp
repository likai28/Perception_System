// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han

#include "calibration_parser.h"
#include <iostream>
#include <fstream>

CalibrationParser::CalibrationParser()
{

}


CalibrationParser::~CalibrationParser()
{

}


bool CalibrationParser::loadCalibration(const std::string filePath)
{
	std::ifstream calibrationContent(filePath.c_str());
	if (calibrationContent.fail()) {
		std::cerr << "Cannot load calibration file at " << filePath << std::endl;
		return false;
	}

	std::string line;
	std::string separator;

	while (true) {
		std::getline(calibrationContent, line);
		if (line.empty()) {
			break;
		}

		// Load camera name
		std::string name = line;

		// Load intrinsic matrix
		Eigen::Matrix3d intrinsicMatrix;
		calibrationContent
			>> intrinsicMatrix(0, 0) >> intrinsicMatrix(0, 1) >> intrinsicMatrix(0, 2)
			>> intrinsicMatrix(1, 0) >> intrinsicMatrix(1, 1) >> intrinsicMatrix(1, 2)
			>> intrinsicMatrix(2, 0) >> intrinsicMatrix(2, 1) >> intrinsicMatrix(2, 2);

		// Load distortion matrix
		Eigen::MatrixXd distortionMatrix(1, 5);
		calibrationContent
			>> distortionMatrix(0, 0) >> distortionMatrix(0, 1) >> distortionMatrix(0, 2)
			>> distortionMatrix(0, 3) >> distortionMatrix(0, 4);

		// Load extrinsic matrix
		Eigen::MatrixXd extrinsicMatrix(3, 4);
		calibrationContent
			>> extrinsicMatrix(0, 0) >> extrinsicMatrix(0, 1) >> extrinsicMatrix(0, 2) >> extrinsicMatrix(0, 3)
			>> extrinsicMatrix(1, 0) >> extrinsicMatrix(1, 1) >> extrinsicMatrix(1, 2) >> extrinsicMatrix(1, 3)
			>> extrinsicMatrix(2, 0) >> extrinsicMatrix(2, 1) >> extrinsicMatrix(2, 2) >> extrinsicMatrix(2, 3);

		// Store calibration parameters
		camNames_.push_back(name);
		intrinsics_.push_back(intrinsicMatrix);
		extrinsics_.push_back(extrinsicMatrix);
		distortions_.push_back(distortionMatrix);
		projections_.push_back(intrinsicMatrix * extrinsicMatrix);

		std::getline(calibrationContent, separator);
		std::getline(calibrationContent, separator);
	}
	return true;
}


bool CalibrationParser::matchCameras(const std::vector<std::string> camNames)
{
	std::vector<std::string> newCamNames;
	std::vector<Eigen::Matrix3d> newIntrinsics;
	std::vector<Eigen::MatrixXd> newExtrinsics;
	std::vector<Eigen::MatrixXd> newDistortions;
	std::vector<Eigen::MatrixXd> newProjections;

	// Brute force search to match calibration parameters to camera names
	for (int i = 0; i < camNames.size(); i++) {
		int index = -1;
		for (int j = 0; j < camNames_.size(); j++) {
			if (camNames_[j] == camNames[i]) {
				index = j;
				break;
			}
		}

		// Failed to match an input camera
		if (index == -1) {
			std::cerr << "Camera name does not exist. Please check your calibration file." << std::endl;
			return false;
		}
		newCamNames.push_back(camNames_[index]);
		newIntrinsics.push_back(intrinsics_[index]);
		newExtrinsics.push_back(extrinsics_[index]);
		newDistortions.push_back(distortions_[index]);
		newProjections.push_back(intrinsics_[index] * extrinsics_[index]);
	}

	std::cout << camNames_.size() << " cameras found. Using " << newCamNames.size() << " cameras." << std::endl;

	// Update camera parameters
	camNames_ = newCamNames;
	intrinsics_ = newIntrinsics;
	extrinsics_ = newExtrinsics;
	distortions_ = newDistortions;
	projections_ = newProjections;

	return true;
}


std::vector<std::string> CalibrationParser::getCamNames()
{
	return camNames_;
}


std::vector<Eigen::Matrix3d> CalibrationParser::getIntrinsics()
{
	return intrinsics_;
}


std::vector<Eigen::MatrixXd> CalibrationParser::getExtrinsics()
{
	return extrinsics_;
}


std::vector<Eigen::MatrixXd> CalibrationParser::getDistortions()
{
	return distortions_;
}


std::vector<Eigen::MatrixXd> CalibrationParser::getProjections()
{
	return projections_;
}


void CalibrationParser::clear()
{
	camNames_.clear();
	intrinsics_.clear();
	extrinsics_.clear();
	projections_.clear();
}
