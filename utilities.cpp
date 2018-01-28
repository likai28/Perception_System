// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han

#include "utilities.h"

int printRSContextInfo(rs::context *c)
{
	printf("There are %d connected RealSense devices.\n", c->get_device_count());
	if (c->get_device_count() == 0) {
		throw std::runtime_error("No device detected. Is it plugged in?");
	}

	return 0;
}



void initRSCamera(
	rs::device *dev,
	const bool depth
)
{

	dev->enable_stream(rs::stream::color, rs::preset::best_quality);
	if (depth)
	{
		dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
		//dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
	}
	dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);
	dev->set_option(rs::option::f200_laser_power, 16);

	//dev->set_option(rs::option::color_enable_auto_exposure, 1.0);
	dev->set_option(rs::option::color_exposure, -1.0);
	dev->set_option(rs::option::color_enable_auto_white_balance, 1.0);
	dev->start();
}


void getStreamingData(
	rs::device* dev,
	uint8_t *colorImg,
	uint16_t *depthImg
)
{
	rs::intrinsics depthIntrin = dev->get_stream_intrinsics(rs::stream::depth);
	rs::intrinsics colorIntrin = dev->get_stream_intrinsics(rs::stream::color);

	if (dev->is_streaming()) {
		dev->wait_for_frames();
	}

	// Get depth image
	const uint16_t* depthPtr = (const uint16_t*)dev->get_frame_data(rs::stream::depth);
	int depthSize = depthIntrin.height * depthIntrin.width;
	depthImg = (uint16_t*)malloc(sizeof(uint16_t) * depthSize);
	std::copy(
		depthPtr,
		depthPtr + depthSize,
		depthImg
	);

	// Get color image
	const uint8_t* colorPtr = (const uint8_t*)dev->get_frame_data(rs::stream::color);
	int colorSize = colorIntrin.height * colorIntrin.width;
	colorImg = (uint8_t*)malloc(sizeof(colorImg) * colorSize * 3);
	std::copy(
		colorPtr,
		colorPtr + colorSize * 3,
		colorImg
	);
}


void getStreamingData(
	rs::device* dev,
	uint8_t * colorImg
)
{
	rs::intrinsics colorIntrin = dev->get_stream_intrinsics(rs::stream::color);

	if (dev->is_streaming()) {
		dev->wait_for_frames();
	}

	// Get color image
	const uint8_t* colorPtr = (const uint8_t*)dev->get_frame_data(rs::stream::color);
	int colorSize = colorIntrin.height * colorIntrin.width;
	colorImg = (uint8_t*)malloc(sizeof(colorImg) * colorSize * 3);
	std::copy(
		colorPtr,
		colorPtr + colorSize * 3,
		colorImg
	);
}


void getStreamingData(
	rs::device* dev,
	cv::Mat &colorImg,
	cv::Mat &depthImg
)
{
	rs::intrinsics depthIntrin = dev->get_stream_intrinsics(rs::stream::depth);
	rs::intrinsics colorIntrin = dev->get_stream_intrinsics(rs::stream::color);

	if (dev->is_streaming()) {
		dev->wait_for_frames();
	}

	// Get depth image
	colorImg = cv::Mat(
		cv::Size(colorIntrin.width, colorIntrin.height),
		CV_8UC3,
		(uchar*)dev->get_frame_data(rs::stream::color)
	);

	// Get color image
	depthImg = cv::Mat(
		cv::Size(depthIntrin.width, depthIntrin.height),
		CV_16U,
		(uchar*)dev->get_frame_data(rs::stream::depth)
	);
}


void getStreamingData(
	rs::device* dev,
	cv::Mat &colorImg
)
{
	rs::intrinsics colorIntrin = dev->get_stream_intrinsics(rs::stream::color);

	if (dev->is_streaming()) {
		dev->wait_for_frames();
	}

	// Get color image
	colorImg = cv::Mat(
		cv::Size(colorIntrin.width, colorIntrin.height),
		CV_8UC3,
		(uchar*)dev->get_frame_data(rs::stream::color)
	);
}


void mergeChannelMasks(
	const cv::Mat &channelMasks,
	cv::Mat &mask
)
{
	std::vector<cv::Mat> channels;
	cv::split(channelMasks, channels);
	mask = channels[0].clone();
	for (int i = 1; i < channels.size(); i++) {
		cv::multiply(mask, channels[i], mask);
	}
}


void maskImage(
	const cv::Mat &mask,
	cv::Mat &img
)
{
	assert(mask.size() == img.size());

	std::vector<cv::Mat> channels;
	cv::split(img, channels);
	for (int i = 0; i < channels.size(); i++) {
		cv::multiply(channels[i], mask, channels[i]);
	}
	cv::merge(channels, img);
}


void adaptiveThreshold(
	const cv::Mat &img,
	const int blockSize,
	const double constant,
	const int thresholdType,
	cv::Mat &mask
)
{
	if (img.type() != CV_8UC1 && img.type() != CV_8UC3) {
		if (img.channels() == 1) {
			cv::cvtColor(img, img, CV_8UC1);
		}
		else {
			cv::cvtColor(img, img, CV_8UC3);
		}
	}

	mask = cv::Mat::ones(img.size(), CV_32FC1);
	std::vector<cv::Mat> channels;
	cv::split(img, channels);
	for (int i = 0; i < channels.size(); i++) {
		cv::adaptiveThreshold(
			channels[i],
			channels[i],
			1.0,
			cv::ADAPTIVE_THRESH_GAUSSIAN_C,
			thresholdType,
			blockSize,
			constant
		);

		cv::multiply(
			mask,
			channels[i],
			mask,
			1.0,
			CV_32FC1
		);
	}
}


cv::Vec3b getJetColor(
	const double val
)
{
	cv::Mat gray(cv::Size(1, 1), CV_64FC1);
	gray.at<double>(0, 0) = val;
	gray.convertTo(gray, CV_8UC1, 255);
	cv::applyColorMap(gray, gray, cv::COLORMAP_JET);
	return gray.at<cv::Vec3b>(0, 0);
}