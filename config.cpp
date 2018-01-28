// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han

#include "config.h"
#include <iostream>
#include <fstream>

Config::Config()
{
	// Default config parameters
	waitTimeSec_ = 3;

	nOctaves_ = 4;
	nOctaveLayers_ = 2;
	hessianThreshold_ = 700.0;

	cpuOnly_ = false;
	modelFile_ = "AlexNet-Siamese_from_scratch_iter_6000.caffemodel";
	deployFile_ = "Alexnet-Siamese.prototxt";
	meanFile_ = "alexnet_mean.binaryproto";
	templateFile_ = ".\\templates";

	colorCalibrationFile_ = "calibration_rgb.txt";
	depthCalibrationFile_ = "calibration_depth.txt";
}


Config::~Config()
{

}


bool Config::init(const std::string configFile)
{
	std::ifstream configContent(configFile.c_str());
	if (configContent.fail()) {
		std::cerr << "Fail to load config file at " << configFile << std::endl;
		return false;
	}

	std::cout << "Parsing config from " << configFile << std::endl;
	std::string line;
	while (true) {
		if (!(configContent >> line)) {
			break;
		}

		if (line == "#waitTimeSec") {
			configContent >> waitTimeSec_;
			std::cout << "waitTimeSec: " << waitTimeSec_ << std::endl;

		}
		else if (line == "#octave") {
			configContent >> nOctaves_;
			std::cout << "nOctaves: " << nOctaves_ << std::endl;

		}
		else if (line == "#nOctaveLayers") {
			configContent >> nOctaveLayers_;
			std::cout << "nOctaveLayers: " << nOctaveLayers_ << std::endl;

		}
		else if (line == "#hessianThreshold") {
			configContent >> hessianThreshold_;
			std::cout << "hessianThreshold: " << hessianThreshold_ << std::endl;

		}
		else if (line == "#thresDist") {
			configContent >> thresDist_;
			std::cout << "thresDist: " << thresDist_ << std::endl;

		}
		else if (line == "#batchSize") {
			configContent >> batchSize_;
			std::cout << "batchSize: " << batchSize_ << std::endl;

		}
		else if (line == "#cpuOnly") {
			configContent >> cpuOnly_;
			std::cout << "cpuOnly: " << cpuOnly_ << std::endl;

		}
		else if (line == "#modelFile") {
			configContent >> modelFile_;
			std::cout << "modelFile: " << modelFile_ << std::endl;

		}
		else if (line == "#deployFile") {
			configContent >> deployFile_;
			std::cout << "deployFile: " << deployFile_ << std::endl;

		}
		else if (line == "#meanFile") {
			configContent >> meanFile_;
			std::cout << "meanFile: " << meanFile_ << std::endl;

		}
		else if (line == "#templateFile") {
			configContent >> templateFile_;
			std::cout << "templateFile: " << templateFile_ << std::endl;

		}
		else if (line == "#useSiamese") {
			configContent >> useSiamese_;
			std::cout << "useSiamese: " << useSiamese_ << std::endl;

		}
		else if (line == "#colorCalibrationFile") {
			configContent >> colorCalibrationFile_;
			std::cout << "colorCalibrationFile: " << colorCalibrationFile_ << std::endl;

		}
		else if (line == "#depthCalibrationFile") {
			configContent >> depthCalibrationFile_;
			std::cout << "depthCalibrationFile: " << depthCalibrationFile_ << std::endl;
		}
	}
	return true;
}