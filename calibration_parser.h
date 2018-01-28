// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han

#ifndef CALIBRATION_PARSER_H
#define CALIBRATION_PARSER_H

#include <Eigen/Core>
#include <vector>

class CalibrationParser
{

public:

	CalibrationParser();
	~CalibrationParser();

	bool loadCalibration(const std::string filePath);				// Load calibration parameters from input file
	bool matchCameras(const std::vector<std::string> camNames);		// Match loaded calibration parameters to given camera names
	std::vector<std::string> getCamNames();							// Get camera names
	std::vector<Eigen::Matrix3d> getIntrinsics();					// Get intrinsic matrices
	std::vector<Eigen::MatrixXd> getExtrinsics();					// Get extrinsic matrices
	std::vector<Eigen::MatrixXd> getDistortions();					// Get distortion matrices
	std::vector<Eigen::MatrixXd> getProjections();					// Get projection matrices
	void clear();													// Clear all internal variables


private:

	std::vector<std::string> camNames_;								// Camera names
	std::vector<Eigen::Matrix3d> intrinsics_;						// Intrinsic parameters
	std::vector<Eigen::MatrixXd> extrinsics_;						// Extrinsic parameters
	std::vector<Eigen::MatrixXd> distortions_;						// Distortion parameters
	std::vector<Eigen::MatrixXd> projections_;						// Projection parameters
};

#endif