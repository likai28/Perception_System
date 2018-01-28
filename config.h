// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han

#ifndef CONFIG_H
#define CONFIG_H

#include <string>

struct Config
{
	Config();
	~Config();
	bool init(const std::string configFile);

	int waitTimeSec_;												// Number of seconds for lifting hand after grasping

	int nOctaves_;													// Number of octaves for SURF feature detector
	int nOctaveLayers_;												// Number of levels for SURF feature detector
	double hessianThreshold_;										// The Hessian threshold for SURF feature detector

	double thresDist_;												// Threshold for feature distance between Siamese Network outputs
	int batchSize_;													// Input batch size for prediction
	bool cpuOnly_;													// Whether to use CPU mode of caffe model
	bool useSiamese_;												// Whether to use Siamese network for image patch matching
	std::string modelFile_;											// Pre-trained caffe model path
	std::string deployFile_;										// Deloy prototxt of caffe model
	std::string meanFile_;											// Mean file for caffe model
	std::string templateFile_;										// Tempate files of finger markers

	std::string colorCalibrationFile_;								// Calibration file path
	std::string depthCalibrationFile_;								// Calibration file path
};


#endif

