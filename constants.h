// Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// By Wentao Han & Kai Li

#ifndef CONSTANTS_H
#define CONSTANTS_H

// Modes of the capture workflow
enum { MOTION_DETECTION_MODE = 0, CAPTURE_MODE = 1, EXIT_MODE = 2 };

// Constants used for RS cameras
#define MAIN_CAM_ID 0
#define DEPTH_FOREGROUND_THRES 60.0
#define MOTION_THRES 100.0
#define MOTION_THRES_RATIO 0.10
#define STILL_THRES_RATIO 0.10
#define STILL_THRES_NUM_FRAMES 180
#define BACKGROUND_HISTORY 30
#define RS_INFRARED_MAX_POWER 8
#define RS_INFRARED_MIN_POWER 0
#define RS_INFRARED_WAIT_TIME 100

// Constants used for marker detection
#define THUMB_CLASS_ID 0
#define MAX_BLOB_SIZE 60
#define MIN_BLOB_SIZE 20
#define NUM_CLASSES 5
#define CROP_SCALE 2.5
#define THUMB_AREA_THRES_RATIO 0.20
#define CENTER_AREA_THRES_RATIO 0.20
#define BLACK_AREA_THRES 60.0
#define WHITE_AREA_THRES 110.0
#define THUMB_WHITE_AREA_THRES 90.0
#define REFINE_CROP_SCALE 1.2
#define THUMB_REFINE_CROP_SCALE 2.0
#define ADAPTIVE_THRES_SIZE 7
#define CONTOUR_BBOX_THRES_RATIO 0.9

// Constants used for reconstruction and deprojection
#define MIN_PROJECT_ERR 4.0
#define REG_MAX_CORRES_DIST 0.0025
#define MILLI_TO_METER_SCALE 0.001
#define REG_MAX_SEARCH_NEIGHBOR 15
#define REG_MIN_EPSILON 1e-5
#define REG_MAX_ITER 30
#define TOUCH_POINT_RADIUS 0.005

#define ONE_FINGER_LOW_BOUND 45
#define ONE_FINGER_HIGH_BOUND 1500
#define TWO_FINGER_HIGH_BOUND 3300
#define THREE_FINGER_HIGH_BOUND 7800


#endif
