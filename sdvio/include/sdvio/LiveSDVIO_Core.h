#pragma once

#include <global.h>
#include <sensor_msgs/Imu.h>
#include <imageMeasurement.h>
#include <imuMeasurement.h>
#include <frameHandler.h>

namespace sdvio 
{
typedef vector<pair<vector<ImuMeasurement>, pair<cv::Mat, cv::Mat>>> MeasurementPairs;

class LiveSdvioCore 
{	
public:  
    FrameHandler				frame_handler;
    std::condition_variable 			con;
    std::mutex					mea_queue_mtx;
    std::list<ImageMeasurement> 		image0_buf;
    boost::mutex				image0_queue_mtx;
    std::list<ImageMeasurement> 		image1_buf;
    boost::mutex 				image1_queue_mtx;    
    std::list<ImuMeasurement>			imu_buf;
    boost::mutex 				imu_queue_mtx;  
    
    Eigen::Vector3d 				last_acc;
    Eigen::Vector3d 				last_gyr;
    Eigen::Vector3d 				last_ba;
    Eigen::Vector3d 				last_bg;
    int 					frame_counter;
    
    LiveSdvioCore(ros::NodeHandle& pnh);
    ~LiveSdvioCore();
    
    void Loop();
    void getMeasurementPairs(MeasurementPairs& measurement_pairs);
    
private:
    
};

}
