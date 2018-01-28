// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han

#ifndef UTILITIES_H
#define UTILITIES_H

#include <opencv2/opencv.hpp>
#include <librealsense/rs.hpp>
#include "constants.h"

int printRSContextInfo(rs::context *c);
// Print RealSense device info


void initRSCamera(
	rs::device *dev,
	const bool depth = false
);	// Initialize RealSense camera: enable and start streaming



void getStreamingData(
	rs::device* dev,
	uint8_t * colorImg,
	uint16_t* depthImg
);


void getStreamingData(
	rs::device* dev,
	uint8_t * colorImg
);


void getStreamingData(
	rs::device* dev,
	cv::Mat &colorImg,
	cv::Mat &depthImg
);


void getStreamingData(
	rs::device* dev,
	cv::Mat &colorImg
);


void mergeChannelMasks(
	const cv::Mat &channelMasks,
	cv::Mat &mask
);


void maskImage(
	const cv::Mat &mask,
	cv::Mat &img
);


void adaptiveThreshold(
	const cv::Mat &img,
	const int blockSize,
	const double constant,
	const int thresholdType,
	cv::Mat &mask
);


cv::Vec3b getJetColor(
	const double val
);

#endif
