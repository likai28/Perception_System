// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Kai Li

#ifndef THERMAL_MODULES_H
#define THERMAL_MODULES_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <ctime>
#include <string>
#include <algorithm>
#include <functional>
#include <array>
#include <Eigen/Dense>
#include "visual_modules.h"

std::string theramlCapture();

int findFingerNum(cv::Mat &img);

cv::Mat kmeansClustering(cv::Mat &image, int clusterCount);

std::vector<cv::Mat> thermalSegmentation();

void thermalReproject(std::vector<rs::device*> &rsCameras, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds, const std::vector<cv::Mat> &centers);


#endif