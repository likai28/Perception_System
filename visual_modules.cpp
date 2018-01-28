// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han


#include "visual_modules.h"


void initMultiBackgroundImg(
	std::vector<rs::device*> rsCameras,
	std::vector<cv::Mat> &backgroundImg
)
{
	// Estimate background by averaging frames within a time range
	backgroundImg.resize(rsCameras.size());

	// Turn off all infrared projectors
	for (int i = 0; i < rsCameras.size(); i++) {
		rsCameras[i]->set_option(rs::option::f200_laser_power, RS_INFRARED_MIN_POWER);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME * 2));

	for (int i = 0; i < rsCameras.size(); i++) {
		rsCameras[i]->set_option(rs::option::f200_laser_power, RS_INFRARED_MAX_POWER);
		std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME));
		// Take turns to get depth map
		for (int h = 0; h < BACKGROUND_HISTORY; h++) {
			cv::Mat depth;
			getStreamingData(
				rsCameras[i],
				cv::Mat(),
				depth
			);
			depth.convertTo(depth, CV_32FC1);

			// Update background
			if (backgroundImg[i].empty()) {
				backgroundImg[i] = depth.clone();
			}
			else {
				cv::add(backgroundImg[i], depth, backgroundImg[i]);
			}
		}
		backgroundImg[i] /= BACKGROUND_HISTORY;
		rsCameras[i]->set_option(rs::option::f200_laser_power, RS_INFRARED_MIN_POWER);
		std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME));
	}
}


bool motionDetectionInteration(
	const cv::Mat &backgroundImg,
	std::vector<rs::device*> &rsCameras
)
{
	cv::Mat depth;
	getStreamingData(
		rsCameras[MAIN_CAM_ID],
		cv::Mat(),
		depth
	);
	depth.convertTo(depth, CV_32FC1);

	// Detect motion by thresholding frame difference
	float depthThres = depth.rows * depth.cols * MOTION_THRES_RATIO;
	cv::Mat depthDiff;
	cv::absdiff(backgroundImg, depth, depthDiff);

	cv::Mat depthDiffBin;
	cv::threshold(depthDiff, depthDiffBin, MOTION_THRES, 1.0, cv::THRESH_BINARY);

	float motion = cv::sum(depthDiffBin)[0];
	if (motion > depthThres) {
		return true;
	}
	else {
		return false;
	}
}


void captureIteration(
	const std::vector<cv::Mat> &backgroundImgs,
	const std::string tempPath,
	std::vector<rs::device*> &rsCameras,
	cv::Mat &prevFrame,
	int &numStill
)
{
	// Detect still image with the main camera
	cv::Mat depth;
	getStreamingData(
		rsCameras[MAIN_CAM_ID],
		cv::Mat(),
		depth
	);
	depth.convertTo(depth, CV_32FC1);

	if (prevFrame.empty()) {
		prevFrame = depth.clone();
	}
	else {
		float thresMotion = depth.cols * depth.rows * STILL_THRES_RATIO;
		cv::Mat depthDiff;
		cv::Mat depthDiffPrev;

		cv::subtract(backgroundImgs[MAIN_CAM_ID], depth, depthDiff);
		cv::subtract(backgroundImgs[MAIN_CAM_ID], prevFrame, depthDiffPrev);

		prevFrame = depth.clone();
		cv::threshold(depthDiff, depthDiff, 0.0, 1.0, cv::THRESH_BINARY);
		cv::threshold(depthDiffPrev, depthDiffPrev, 0.0, 1.0, cv::THRESH_BINARY);

		cv::Mat frameDiff;
		cv::absdiff(depthDiff, depthDiffPrev, frameDiff);
		float motion = cv::sum(frameDiff)[0];

		if (motion < thresMotion) {
			numStill++;
		}
		else {
			numStill = 0;
		}
	}

	// Take turns to capture RGB images and depth maps
	if (numStill >= STILL_THRES_NUM_FRAMES) {
		// Start capturing
		for (int i = 0; i < rsCameras.size(); i++) {
			cv::Mat bgr;
			getStreamingData(
				rsCameras[i],
				bgr
			);

			// Save retrieved images to a temporary direcory
			cv::imwrite(tempPath + "cam" + std::to_string(i) + "\\image.png", bgr);
		}
	}
}


void generatePointCloud(
	const Eigen::Matrix3d &colorIntrinsic,
	const Eigen::Matrix3d &depthIntrinsic,
	const Eigen::MatrixXd &colorToDepthExtrinsic,
	const cv::Mat &backgroundImg,
	const cv::Mat &depthImg,
	const cv::Mat &colorImg,
	rs::device *dev,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
)
{
	// Assign calibrate parameters, instead of using built-in ones
	rs::intrinsics depthIntrin = dev->get_stream_intrinsics(rs::stream::depth);
	rs::extrinsics depthToColor = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
	rs::intrinsics colorIntrin = dev->get_stream_intrinsics(rs::stream::color);

	colorIntrin.fx = colorIntrinsic(0, 0);
	colorIntrin.fy = colorIntrinsic(1, 1);
	colorIntrin.ppx = colorIntrinsic(0, 2);
	colorIntrin.ppy = colorIntrinsic(1, 2);

	depthIntrin.fx = depthIntrinsic(0, 0);
	depthIntrin.fy = depthIntrinsic(1, 1);
	depthIntrin.ppx = depthIntrinsic(0, 2);
	depthIntrin.ppy = depthIntrinsic(1, 2);

	depthToColor.rotation[0] = colorToDepthExtrinsic(0, 0);
	depthToColor.rotation[1] = colorToDepthExtrinsic(0, 1);
	depthToColor.rotation[2] = colorToDepthExtrinsic(0, 2);
	depthToColor.rotation[3] = colorToDepthExtrinsic(1, 0);
	depthToColor.rotation[4] = colorToDepthExtrinsic(1, 1);
	depthToColor.rotation[5] = colorToDepthExtrinsic(1, 2);
	depthToColor.rotation[6] = colorToDepthExtrinsic(2, 0);
	depthToColor.rotation[7] = colorToDepthExtrinsic(2, 1);
	depthToColor.rotation[8] = colorToDepthExtrinsic(2, 2);

	depthToColor.translation[0] = colorToDepthExtrinsic(0, 3) * MILLI_TO_METER_SCALE;
	depthToColor.translation[1] = colorToDepthExtrinsic(1, 3) * MILLI_TO_METER_SCALE;
	depthToColor.translation[2] = colorToDepthExtrinsic(2, 3) * MILLI_TO_METER_SCALE;

	// Retrieve RGB and depth images
	cv::Mat depth = depthImg.clone();
	depth.convertTo(depth, CV_32FC1);

	// Generate object mask
	cv::Mat depthDiff;
	cv::Mat depthMask;
	cv::subtract(backgroundImg, depth, depthDiff);
	cv::threshold(depth, depth, DEPTH_FOREGROUND_THRES, 1.0, cv::THRESH_BINARY);
	cv::threshold(depthDiff, depthMask, DEPTH_FOREGROUND_THRES, 1.0, cv::THRESH_BINARY);
	cv::multiply(depthMask, depth, depthMask);
	cv::erode(depthMask, depthMask, cv::Mat(), cv::Point(-1, -1), 3);
	cv::dilate(depthMask, depthMask, cv::Mat(), cv::Point(-1, -1), 3);

	// Generate point cloud from RGB image and depth image
	float scale = dev->get_depth_scale();

	for (int dy = 0; dy < depthIntrin.height; dy++) {
		for (int dx = 0; dx < depthIntrin.width; dx++) {
			// Skip points that are masked out
			if (depthMask.at<float>(dy, dx) < 0.5) {
				continue;
			}

			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depthValue = depthImg.at<uint16_t>(dy, dx);
			float depthInMeters = depthValue * scale;

			// Skip over pixels with a depth value of zero, which is used to indicate no data
			if (depthValue == 0) {
				continue;
			}

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
			rs::float2 depthPixel = { static_cast<float>(dx), static_cast<float>(dy) };
			rs::float3 depthPoint = depthIntrin.deproject(depthPixel, depthInMeters);
			rs::float3 colorPoint = depthToColor.transform(depthPoint);
			rs::float2 colorPixel = colorIntrin.project(colorPoint);

			pcl::PointXYZRGB p;
			p.x = depthPoint.x;
			p.y = depthPoint.y;
			p.z = depthPoint.z;

			// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
			const int cx = static_cast<int>(std::round(colorPixel.x));
			const int cy = static_cast<int>(std::round(colorPixel.y));
			if (cx < 0 || cy < 0 || cx >= colorIntrin.width || cy >= colorIntrin.height) {
				p.r = 255;
				p.g = 255;
				p.b = 255;
			}
			else {
				cv::Vec3b color = colorImg.at<cv::Vec3b>(cy, cx);
				p.r = color.val[0];
				p.g = color.val[1];
				p.b = color.val[2];
			}
			cloud->push_back(p);
		}
	}
}


void reconstructPointCloud(
	const std::vector<cv::Mat> &backgroundImgs,
	const std::vector<Eigen::Matrix3d> &colorIntrinsics,
	const std::vector<Eigen::Matrix3d> &depthIntrinsics,
	const std::vector<Eigen::MatrixXd> &colorToDepthExtrinsics,
	const std::vector<Eigen::MatrixXd> &depthToDepthExtrinsics,
	const std::vector<rs::device*> &rsCameras,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds,
	std::string timeStamp
)
{
	// Initialize point clouds
	clouds.resize(rsCameras.size());
	for (int i = 0; i < clouds.size(); i++) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
		clouds[i] = cloudPtr;
	}

	// Turn off all cameras and re-initiaize
	for (int i = 0; i < rsCameras.size(); i++) {
		rsCameras[i]->stop();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME * 10));

	for (int i = 0; i < rsCameras.size(); i++) {
		rsCameras[i]->enable_stream(rs::stream::color, rs::preset::best_quality);
		rsCameras[i]->enable_stream(rs::stream::depth, rs::preset::best_quality);
		rsCameras[i]->set_option(rs::option::f200_laser_power, RS_INFRARED_MIN_POWER);
		rsCameras[i]->set_option(rs::option::color_enable_auto_exposure, 0.0);
		rsCameras[i]->set_option(rs::option::color_exposure, -2.0);
		rsCameras[i]->set_option(rs::option::color_enable_auto_white_balance, 1.0);
		rsCameras[i]->start();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME * 10));

	// Take turns to capture streaming data
	std::vector<cv::Mat> depths(rsCameras.size());
	std::vector<cv::Mat> rgbs(rsCameras.size());
	for (int i = 0; i < rsCameras.size(); i++) {
		rsCameras[i]->set_option(rs::option::f200_laser_power, RS_INFRARED_MAX_POWER);
		std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME));
		getStreamingData(
			rsCameras[i],
			rgbs[i],
			depths[i]
		);
		rsCameras[i]->set_option(rs::option::f200_laser_power, RS_INFRARED_MIN_POWER);
		std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME));
		cv::imwrite(".\\output\\" + timeStamp + "\\original" + std::to_string(i) + ".png", rgbs[i]);
	}

	// Reconstruct point clouds with color and depth images
	omp_set_num_threads(std::min(static_cast<int>(rsCameras.size()), omp_get_num_procs()));
#pragma omp parallel for schedule(static)
	for (int i = 0; i < rsCameras.size(); i++) {
		generatePointCloud(
			colorIntrinsics[i],
			depthIntrinsics[i],
			colorToDepthExtrinsics[i],
			backgroundImgs[i],
			depths[i],
			rgbs[i],
			rsCameras[i],
			clouds[i]
		);
		//pcl::io::savePCDFile(".\\output\\"  + std::to_string(i) + ".pcd", *clouds[i]);
	}
}


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
)
{
	// Load previously generated point cloud
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformedClouds(clouds.size());
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> registeredClouds(clouds.size());
	refinedTransformation.clear();
	refinedTransformation.resize(clouds.size());
	for (int i = 0; i < clouds.size(); i++) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr regPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
		transformedClouds[i] = transPtr;
		registeredClouds[i] = regPtr;
		refinedTransformation[i] = Eigen::Matrix4d::Identity();
	}

	// Apply rigid body transformation on each point cloud
	std::vector<Eigen::Matrix4d> transformations(depthToDepthExtrinsics.size());
	for (int i = 0; i < depthToDepthExtrinsics.size(); i++) {
		transformations[i].block(0, 0, 3, 3) = depthToDepthExtrinsics[i].block(0, 0, 3, 3);
		transformations[i].block(0, 3, 3, 1) = depthToDepthExtrinsics[i].block(0, 3, 3, 1) * MILLI_TO_METER_SCALE;
		transformations[i](3, 0) = 0.0;
		transformations[i](3, 1) = 0.0;
		transformations[i](3, 2) = 0.0;
		transformations[i](3, 3) = 1.0;

		pcl::transformPointCloud(*clouds[i], *transformedClouds[i], transformations[i]);
	}

	// Register multiple point clouds using extrinsic parameters
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempOutput(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (clouds.size() == 1) {
		output = clouds[0];
	}
	else {
		// Find the reference view
		int refView = -1;
		for (int i = 0; i < clouds.size(); i++) {
			if (abs(transformations[i](0, 0) - 1.0) < 1e-6 && \
				abs(transformations[i](1, 1) - 1.0) < 1e-6 && \
				abs(transformations[i](2, 2) - 1.0) < 1e-6) {
				refView = i;
				break;
			}
		}

		// No identity transformation found
		if (refView == -1) {
			refView = 0;
		}

		// Perform registration
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norms;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
		pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;

		norms.setSearchMethod(tree);
		norms.setKSearch(REG_MAX_SEARCH_NEIGHBOR);

		reg.setTransformationEpsilon(REG_MIN_EPSILON);
		reg.setMaxCorrespondenceDistance(REG_MAX_CORRES_DIST);
		reg.setMaximumIterations(REG_MAX_ITER);
		*tempOutput += *transformedClouds[refView];

		for (int i = 0; i < clouds.size(); i++) {
			if (i == refView) {
				continue;
			}
			// Perform registration
			pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalSrc(new pcl::PointCloud<pcl::PointNormal>);
			norms.setInputCloud(transformedClouds[i]);
			norms.compute(*pointsWithNormalSrc);
			pcl::copyPointCloud(*transformedClouds[i], *pointsWithNormalSrc);

			pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalTgt(new pcl::PointCloud<pcl::PointNormal>);
			norms.setInputCloud(tempOutput);
			norms.compute(*pointsWithNormalTgt);
			pcl::copyPointCloud(*tempOutput, *pointsWithNormalTgt);

			reg.setInputSource(pointsWithNormalSrc);
			reg.setInputTarget(pointsWithNormalTgt);

			Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
			pcl::PointCloud<pcl::PointNormal>::Ptr regResult = pointsWithNormalSrc;
			reg.align(*regResult);
			Ti = reg.getFinalTransformation().cast<double>() * Ti;
			refinedTransformation[i] = Ti;
			pcl::transformPointCloud(*transformedClouds[i], *registeredClouds[i], Ti);
			*tempOutput += *registeredClouds[i];
		}
	}

	*output = *tempOutput;

	pcl::io::savePLYFileASCII(outputPath + timeStamp + "_original.ply", *output);
}


void executeMode(
	const std::vector<cv::Mat> &backgroundImgs,
	const std::string &tempPath,
	const int waitTimeSec,
	std::vector<rs::device*> &rsCameras,
	int mode,
	int &numStill
)
{
	rsCameras[MAIN_CAM_ID]->set_option(rs::option::f200_laser_power, RS_INFRARED_MAX_POWER);
	std::this_thread::sleep_for(std::chrono::milliseconds(RS_INFRARED_WAIT_TIME));
	cv::Mat prevFrame;

	while (true) {
		// Motion detection mode
		if (MOTION_DETECTION_MODE == mode) {
			bool result = motionDetectionInteration(
				backgroundImgs[MAIN_CAM_ID],
				rsCameras
			);

			// Motion detected, Switch to capture mode
			if (result) {
				mode = CAPTURE_MODE;
				std::cout << "Motion detected. Switching to capture mode..." << std::endl;
			}
		}
		else if (CAPTURE_MODE == mode) {
			// Capture mode
			captureIteration(
				backgroundImgs,
				tempPath,
				rsCameras,
				prevFrame,
				numStill
			);

			// Still image detected
			if (numStill >= STILL_THRES_NUM_FRAMES) {

				mode = EXIT_MODE;
				std::cout << "Please lift your hand in " + std::to_string(waitTimeSec) + " seconds :)" << std::endl;
			}
		}
		else if (EXIT_MODE == mode) {
			// Data process mode
			std::cout << "Start processing captured data..." << std::endl;
			break;
		}
	}
}