#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <list>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;

extern double ROW, COL;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;
extern Eigen::Vector3d G_w;
extern double SCALE;

extern std::string EX_CALIB_RESULT_PATH;
extern std::string SDVIO_OUTPUT;
extern std::string IMAGE_TOPIC_LEFT;
extern std::string IMAGE_TOPIC_RIGHT;
extern std::string IMU_TOPIC;

extern int MAX_CNT;
extern int MIN_DIST;
extern int FREQ;
extern double F_THRESHOLD;
extern double MIN_PARALLAX;
extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;

extern int ESTIMATE_EXTRINSIC;
extern bool PUB_THIS_FRAME;

extern std::vector<std::string> CAM_NAMES;
extern camodocal::CameraPtr CAM_LEFT;
extern camodocal::CameraPtr CAM_RIGHT;

void readParameters(ros::NodeHandle &n);
void readIntrinsicParameter(const std::string &calib_file);

extern int KLT_MAX_LEVEL;
extern int NUM_PYR_LEVELS;

extern double MIN_CORNER_SCORE;
extern double TRIANG_MIN_CORNER_SCORE;
extern int GRID_SIZE;

namespace sdvio 
{
    using namespace Eigen;
    using namespace Sophus;
    using namespace std;   
    using namespace camodocal;
    
    class Frame;
    typedef boost::shared_ptr<Frame> FramePtr;    
    
    const double EPS = 0.0000000001;
    const double PI = 3.14159265;
}

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};