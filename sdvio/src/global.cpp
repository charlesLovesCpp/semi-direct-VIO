#include <global.h>

double ROW, COL;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
Eigen::Vector3d G;
Eigen::Vector3d G_w;
double SCALE;
std::string EX_CALIB_RESULT_PATH;
std::string SDVIO_OUTPUT;
std::string IMAGE_TOPIC_LEFT;
std::string IMAGE_TOPIC_RIGHT;
std::string IMU_TOPIC;
int MAX_CNT;
int MIN_DIST;
int FREQ;
double F_THRESHOLD;
double MIN_PARALLAX;
double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
bool PUB_THIS_FRAME;
int ESTIMATE_EXTRINSIC;
std::vector<std::string> CAM_NAMES;
camodocal::CameraPtr CAM_LEFT;
camodocal::CameraPtr CAM_RIGHT;
int KLT_MAX_LEVEL = 3;
int NUM_PYR_LEVELS = 3;
double MIN_CORNER_SCORE = 20.0;
double TRIANG_MIN_CORNER_SCORE = 20.0;
int GRID_SIZE = 30;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;

    config_file = readParam<std::string>(n, "config_file");

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image_topic_left"] >> IMAGE_TOPIC_LEFT;
    fsSettings["image_topic_right"] >> IMAGE_TOPIC_RIGHT;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    
    ROS_INFO_STREAM("IMAGE_TOPIC_LEFT : " << IMAGE_TOPIC_LEFT);
    ROS_INFO_STREAM("IMAGE_TOPIC_RIGHT : " << IMAGE_TOPIC_RIGHT);

    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    
    CAM_NAMES.push_back(config_file);
    
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    
    cv::Mat cv_R, cv_T;
    fsSettings["extrinsicRotation"] >> cv_R;
    fsSettings["extrinsicTranslation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);
    Eigen::Quaterniond Q(eigen_R);
    eigen_R = Q.normalized();
    // image0
    RIC.push_back(eigen_R);
    TIC.push_back(eigen_T);
    // image1
    RIC.push_back(eigen_R);
    TIC.push_back(eigen_T);
    ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
    ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());    

    BIAS_ACC_THRESHOLD = 0.5;
    BIAS_GYR_THRESHOLD = 0.5;

    fsSettings.release();
}

void readIntrinsicParameter(const std::string &calib_file)
{
    ROS_INFO("Reading paramerter of camera %s", calib_file.c_str());
    CAM_LEFT = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    CAM_RIGHT = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}
