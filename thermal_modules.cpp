// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Kai Li

#include "thermal_modules.h"
#include <iostream>

std::string theramlCapture()
{
	std::vector<cv::Mat> frame;
	frame.resize(4);

	// initialize videocapture
	std::vector<cv::VideoCapture> cap;
	cap.resize(4);
	// open the thermal camera using updated URL 
	// Needs updating every time you unplug or plug the thermal connection
	// Also check the corespendence between URLs and theraml cameras
	cap[0].open("rtsp://169.254.118.50/mpeg4?overlay=off");
	cap[1].open("rtsp://169.254.34.206/mpeg4?overlay=off");
	cap[2].open("rtsp://169.254.132.87/mpeg4?overlay=off");
	cap[3].open("rtsp://169.254.141.65/mpeg4?overlay=off");
	cap[0].set(CV_CAP_PROP_BUFFERSIZE, 0);
	cap[1].set(CV_CAP_PROP_BUFFERSIZE, 0);
	cap[2].set(CV_CAP_PROP_BUFFERSIZE, 0);
	cap[3].set(CV_CAP_PROP_BUFFERSIZE, 0);

	// check if we succeeded in open cap
	for (int num = 0; num<4; num++)
		if (!cap[num].isOpened()) {
			std::cout << "ERROR! Unable to open camera " << num << std::endl;
		}

	std::vector<std::string> str;
	str.resize(4);

	for (int num = 0; num < 4; num++)
	{
		for (int i = 0; i < 40; i++) {
			cap[num] >> frame[num];
			// check if we succeeded in grasping the frames
			if (frame[num].empty()) {
				std::cout << "ERROR! blank frame grabbed\n";
				break;
			}
		}
	}

	str[0] = ".\\th1\\";
	str[1] = ".\\th2\\";
	str[2] = ".\\th3\\";
	str[3] = ".\\th4\\";

	time_t time = std::time(nullptr);
	std::string timeStamp(std::to_string((int)time));
	for (int num = 0; num < 4; num++) {
		std::string final = str[num] + timeStamp + "_" + std::to_string(num) + ".png";
		imwrite(final, frame[num]);
	}
	return timeStamp;
}

int findFingerNum(cv::Mat &img)
{
	cv::Mat gray;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::RNG rng(12345);
	cv::Mat drawing;
	cv::cvtColor(img, gray, CV_BGR2GRAY);
	cv::Canny(gray, gray, 200, 400, 3);

	//Find contours   
	cv::findContours(gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	drawing = cv::Mat::zeros(gray.size(), CV_8UC3);
	if (contours.size() == 0)
	{
		return 0;
	}
	std::vector<cv::Moments> mu(contours.size());
	std::vector<cv::Rect> boundRect(contours.size());
	std::vector<int> boundingarea;
	std::vector<double> contoursize(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		contoursize[i] = cv::contourArea(contours[i]);
		boundRect[i] = cv::boundingRect(contours[i]);
		boundingarea.push_back(boundRect[i].width * boundRect[i].height);
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		mu[i] = moments(contours[i], false);
	}

	//imshow("Contour initially", drawing);
	//cv::waitKey(0);
	int finger_count = 0;
	for (int i = 0; i < boundingarea.size(); i++)
	{
		std::cout << "Initially bounding area size for " << " " << i << ", " << boundingarea[i] << std::endl;
	}
	if (boundingarea.size() > 1)
	{
		sort(boundingarea.begin(), boundingarea.end(), std::greater<double>());
	}
	for (int i = 0; i < boundingarea.size(); i++)
	{
		std::cout << "Reordered bounding area size for " << " " << i << ", " << boundingarea[i] << std::endl;
		if (boundingarea[i] > ONE_FINGER_LOW_BOUND && boundingarea[i] < ONE_FINGER_HIGH_BOUND)
		{
			finger_count += 1;
		}
		else if (boundingarea[i] >= ONE_FINGER_HIGH_BOUND && boundingarea[i] < TWO_FINGER_HIGH_BOUND)
		{
			finger_count += 2;
		}
		else if (boundingarea[i] >= TWO_FINGER_HIGH_BOUND && boundingarea[i] < THREE_FINGER_HIGH_BOUND)
		{
			finger_count += 3;
		}
		else if (boundingarea[i] >= THREE_FINGER_HIGH_BOUND)
		{
			finger_count += 4;
		}
	}
	return std::min(finger_count, 5);
}

cv::Mat kmeansClustering(cv::Mat &image, int clusterCount)
{
	std::vector<cv::Point2f> points;
	cv::Mat labels; cv::Mat centers;
	for (int i = 0; i < image.rows; i++)
	{
		for (int j = 0; j < image.cols; j++)
		{
			if ((double)image.at<cv::Vec3b>(i, j)[2] > 250)
			{
				cv::Point2f pixel_index;
				pixel_index.x = j;
				pixel_index.y = i;
				points.push_back(pixel_index);
			}
		}
	}
	if (!points.empty() && clusterCount>0)
	{
		cv::kmeans(points, clusterCount, labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), 1, cv::KMEANS_PP_CENTERS, centers);
	}

	return centers;
}


std::vector<cv::Mat> thermalSegmentation()
{
	std::vector<std::string> background(4);
	background[0] = "heat_background1.png";
	background[1] = "heat_background2.png";
	background[2] = "heat_background3.png";
	background[3] = "heat_background4.png";

	std::string timestamp = theramlCapture();

	std::vector<std::string> files_name(4);
	std::vector<std::string> files_location(4);
	std::vector<std::string> windows(4);
	files_location[0] = ".\\th1\\";
	files_location[1] = ".\\th2\\";
	files_location[2] = ".\\th3\\";
	files_location[3] = ".\\th4\\";
	windows[0] = "windows1";
	windows[1] = "windows2";
	windows[2] = "windows3";
	windows[3] = "windows4";
	cv::namedWindow(windows[0], 1);
	cv::moveWindow(windows[0], 0, 0);
	cv::namedWindow(windows[1], 1);
	cv::moveWindow(windows[1], 650, 0);
	cv::namedWindow(windows[2], 1);
	cv::moveWindow(windows[2], 0, 400);
	cv::namedWindow(windows[3], 1);
	cv::moveWindow(windows[3], 650, 400);

	for (int i = 0; i < 4; i++)
	{
		files_name[i] = files_location[i] + timestamp + "_" + std::to_string(i) + ".png";
	}
	std::vector<cv::Mat> cool(4);
	std::vector<cv::Mat> heat(4);
	std::vector<cv::Mat> diffImage(4);
	std::vector<cv::Mat> image(4);
	std::vector<int> clusterCount(4);
	std::vector<cv::Mat> centers(4);
	std::vector<std::string> raw_images_name(4);
	for (int i = 0; i < 4; i++)
	{
		cool[i] = cv::imread(background[i]);
		heat[i] = cv::imread(files_name[i]);
		cv::absdiff(cool[i], heat[i], diffImage[i]);
		cv::threshold(diffImage[i], diffImage[i], 200, 255, cv::THRESH_BINARY);
		cv::erode(diffImage[i], diffImage[i], cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
		image[i] = diffImage[i];
		raw_images_name[i] = files_location[i] + timestamp + "_" + std::to_string(i) + ".jpg";
		cv::imwrite(raw_images_name[i], image[i]);
		//Start K-mean
		clusterCount[i] = findFingerNum(image[i]);
		std::cout << "Detected num of finger on view " << i << " " << "is " << clusterCount[i] << std::endl;
		centers[i] = kmeansClustering(image[i], clusterCount[i]);
		for (int j = 0; j < clusterCount[i]; j++)
		{
			circle(image[i], cv::Point(centers[i].at<float>(j, 0), centers[i].at<float>(j, 1)), 2, cv::Scalar(0, 255, 0), 1, 1, 0);
		}

		imshow(windows[i], image[i]);
		cvWaitKey(1);
	}

	return centers;
}

void thermalReproject(std::vector<rs::device*> &rsCameras, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds,const std::vector<cv::Mat> &centers)
{
	// define the thermal parameters---Extrinsics and Intrinsics
	std::vector<Eigen::MatrixXd> rsToThermalExtrins(rsCameras.size());
	double theta_x; double theta_y; double theta_z; double tx; double ty; double tz;
	std::vector<Eigen::MatrixXd> thermalIntrins(rsCameras.size());
	for (int i = 0; i < rsCameras.size(); i++)
	{
		rsToThermalExtrins[i].resize(3, 4);
		thermalIntrins[i].resize(3, 3);
	}

	// Define the marking color and reprojection vector for heat touching points
	uint8_t r(255), g(0), b(0);
	uint32_t RGB = (static_cast<uint32_t>(r) << 16 |
		static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	Eigen::VectorXd Point(4, 1);
	Eigen::VectorXd Reprojection(3, 1);

	//Match the thermal cameras with their referenced rs cameras and change touching points color in each point cloud
	std::cout << "------------------Matching thermal cameras with 3D point clouds------------------" << std::endl;
	for (int i = 0; i < rsCameras.size(); i++)
	{
		std::string rsName = rsCameras[i]->get_serial();
		if (rsName.compare("614203000681") == 0)
			// currently rs camera 1 for reference
		{
			std::cout << "RS Camera 1 detected, reprojecting 2D heat points to point cloud from RS1..." << std::endl;
			theta_x = 0.3204; theta_y = 0.4783; theta_z = 0.6045; tx = -178.8428; ty = 21.5434; tz = 232.1047;
			rsToThermalExtrins[i] << cos(theta_y)*cos(theta_z), cos(theta_z)*sin(theta_x)*sin(theta_y) - cos(theta_x)*sin(theta_z), sin(theta_x)*sin(theta_z) + cos(theta_x)*cos(theta_z)*sin(theta_y), tx,
				cos(theta_y)*sin(theta_z), cos(theta_x)*cos(theta_z) + sin(theta_x)*sin(theta_y)*sin(theta_z), cos(theta_x)*sin(theta_y)*sin(theta_z) - cos(theta_z)*sin(theta_x), ty,
				-sin(theta_y), cos(theta_y)*sin(theta_x), cos(theta_x)*cos(theta_y), tz;
			thermalIntrins[i] << 720, 0.0, 302,
				0.0, 719, 207,
				0.0, 0.0, 1.0;

			// Map the 2D heatPoints to 3D point clouds accordingly
			for (int k = 0; k < clouds[i]->size(); k++)
			{
				//cout << "Coming into the 3D to 2D verify process!" << endl;
				Point[0] = 1000 * clouds[i]->points[k].x;
				Point[1] = 1000 * clouds[i]->points[k].y;
				Point[2] = 1000 * clouds[i]->points[k].z;
				Point[3] = 1;
				Reprojection = thermalIntrins[i] * rsToThermalExtrins[i] * Point;
				for (int j = 0; j < centers[0].rows; j++)
				{
					if (abs(Reprojection[0] / Reprojection[2] - centers[0].at<float>(j, 0)) < 2 && abs(Reprojection[1] / Reprojection[2] - centers[0].at<float>(j, 1)) < 2)
					{
						clouds[i]->points[k].rgb = *reinterpret_cast<float*>(&RGB);
					}
				}
			}
			if (clouds[i]->size() > 0)
			{
				pcl::io::savePCDFile(".\\output\\" + std::to_string(i) + ".pcd", *clouds[i]);
			}
			else {
				std::cout << "Check your RS camera 1, it cannot generate point cloud properly." << std::endl;
			}
		}
		if (rsName.compare("616205004264") == 0)
			// currently rs camera 2 for reference
		{
			std::cout << "RS Camera 2 detected, reprojecting 2D heat points to point cloud from RS2..." << std::endl;
			theta_x = 0.2956; theta_y = 0.5857; theta_z = 0.6428; tx = -179.7716; ty = -1.0248; tz = 266.1697;
			rsToThermalExtrins[i] << cos(theta_y)*cos(theta_z), cos(theta_z)*sin(theta_x)*sin(theta_y) - cos(theta_x)*sin(theta_z), sin(theta_x)*sin(theta_z) + cos(theta_x)*cos(theta_z)*sin(theta_y), tx,
				cos(theta_y)*sin(theta_z), cos(theta_x)*cos(theta_z) + sin(theta_x)*sin(theta_y)*sin(theta_z), cos(theta_x)*sin(theta_y)*sin(theta_z) - cos(theta_z)*sin(theta_x), ty,
				-sin(theta_y), cos(theta_y)*sin(theta_x), cos(theta_x)*cos(theta_y), tz;
			thermalIntrins[i] << 712.238, 0.0, 297.695,
				0.0, 709.3614, 194.8322,
				0.0, 0.0, 1.0;

			// Map the 2D heatPoints to 3D point clouds accordingly
			for (int k = 0; k < clouds[i]->size(); k++)
			{
				//cout << "Coming into the 3D to 2D verify process!" << endl;
				Point[0] = 1000 * clouds[i]->points[k].x;
				Point[1] = 1000 * clouds[i]->points[k].y;
				Point[2] = 1000 * clouds[i]->points[k].z;
				Point[3] = 1;
				Reprojection = thermalIntrins[i] * rsToThermalExtrins[i] * Point;
				for (int j = 0; j < centers[1].rows; j++)
				{
					if (abs(Reprojection[0] / Reprojection[2] - centers[1].at<float>(j, 0)) < 2 && abs(Reprojection[1] / Reprojection[2] - centers[1].at<float>(j, 1)) < 2)
					{
						clouds[i]->points[k].rgb = *reinterpret_cast<float*>(&RGB);
					}
				}
			}
			if (clouds[i]->size() > 0)
			{
				pcl::io::savePCDFile(".\\output\\" + std::to_string(i) + ".pcd", *clouds[i]);
			}
			else {
				std::cout << "Check your RS camera 2, it cannot generate point cloud properly." << std::endl;
			}
		}

		if (rsName.compare("616205003711") == 0)
			// currently rs camera 3 for reference
		{
			std::cout << "RS Camera 3 detected, reprojecting 2D heat points to point cloud from RS3..." << std::endl;
			theta_x = 0.3808; theta_y = 0.5134; theta_z = 0.6192; tx = -173.1272; ty = 14.5692; tz = 232.3481;
			rsToThermalExtrins[i] << cos(theta_y)*cos(theta_z), cos(theta_z)*sin(theta_x)*sin(theta_y) - cos(theta_x)*sin(theta_z), sin(theta_x)*sin(theta_z) + cos(theta_x)*cos(theta_z)*sin(theta_y), tx,
				cos(theta_y)*sin(theta_z), cos(theta_x)*cos(theta_z) + sin(theta_x)*sin(theta_y)*sin(theta_z), cos(theta_x)*sin(theta_y)*sin(theta_z) - cos(theta_z)*sin(theta_x), ty,
				-sin(theta_y), cos(theta_y)*sin(theta_x), cos(theta_x)*cos(theta_y), tz;
			thermalIntrins[i] << 706.85, 0.0, 328.80,
				0.0, 707.721, 219.28,
				0.0, 0.0, 1.0;

			// Map the 2D heatPoints to 3D point clouds accordingly
			for (int k = 0; k < clouds[i]->size(); k++)
			{
				//cout << "Coming into the 3D to 2D verify process!" << endl;
				Point[0] = 1000 * clouds[i]->points[k].x;
				Point[1] = 1000 * clouds[i]->points[k].y;
				Point[2] = 1000 * clouds[i]->points[k].z;
				Point[3] = 1;
				Reprojection = thermalIntrins[i] * rsToThermalExtrins[i] * Point;
				for (int j = 0; j < centers[2].rows; j++)
				{
					if (abs(Reprojection[0] / Reprojection[2] - centers[2].at<float>(j, 0)) < 2 && abs(Reprojection[1] / Reprojection[2] - centers[2].at<float>(j, 1)) < 2)
					{
						clouds[i]->points[k].rgb = *reinterpret_cast<float*>(&RGB);
					}
				}
			}
			if (clouds[i]->size() > 0)
			{
				pcl::io::savePCDFile(".\\output\\" + std::to_string(i) + ".pcd", *clouds[i]);
			}
			else {
				std::cout << "Check your RS camera 3, it cannot generate point cloud properly." << std::endl;
			}
		}
		if (rsName.compare("615203001586") == 0)
			// currently rs camera 4 for reference
		{
			std::cout << "RS Camera 4 detected, reprojecting 2D heat points to point cloud from RS4..." << std::endl;
			theta_x = 0.2196; theta_y = 0.4642; theta_z = 0.5917; tx = -190.0789; ty = 18.6185; tz = 230.2638;
			rsToThermalExtrins[i] << cos(theta_y)*cos(theta_z), cos(theta_z)*sin(theta_x)*sin(theta_y) - cos(theta_x)*sin(theta_z), sin(theta_x)*sin(theta_z) + cos(theta_x)*cos(theta_z)*sin(theta_y), tx,
				cos(theta_y)*sin(theta_z), cos(theta_x)*cos(theta_z) + sin(theta_x)*sin(theta_y)*sin(theta_z), cos(theta_x)*sin(theta_y)*sin(theta_z) - cos(theta_z)*sin(theta_x), ty,
				-sin(theta_y), cos(theta_y)*sin(theta_x), cos(theta_x)*cos(theta_y), tz;
			thermalIntrins[i] << 716.4285, 0.0, 330.5401,
				0.0, 711.2686, 181.6052,
				0.0, 0.0, 1.0;

			// Map the 2D heatPoints to 3D point clouds accordingly
			for (int k = 0; k < clouds[i]->size(); k++)
			{
				//cout << "Coming into the 3D to 2D verify process!" << endl;
				Point[0] = 1000 * clouds[i]->points[k].x;
				Point[1] = 1000 * clouds[i]->points[k].y;
				Point[2] = 1000 * clouds[i]->points[k].z;
				Point[3] = 1;
				Reprojection = thermalIntrins[i] * rsToThermalExtrins[i] * Point;
				for (int j = 0; j < centers[3].rows; j++)
				{
					if (abs(Reprojection[0] / Reprojection[2] - centers[3].at<float>(j, 0)) < 2 && abs(Reprojection[1] / Reprojection[2] - centers[3].at<float>(j, 1)) < 2)
					{
						clouds[i]->points[k].rgb = *reinterpret_cast<float*>(&RGB);
					}
				}
			}
			if (clouds[i]->size() > 0)
			{
				pcl::io::savePCDFile(".\\output\\" + std::to_string(i) + ".pcd", *clouds[i]);
			}
			else {
				std::cout << "Check your RS camera 4, it cannot generate point cloud properly." << std::endl;
			}
		}

	}
}
