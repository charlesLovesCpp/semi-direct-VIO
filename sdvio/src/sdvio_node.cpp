#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>

#include <global.h>
#include <LiveSDVIO_Core.h>

using namespace sdvio;

ros::Subscriber sub_image[2];
ros::Subscriber sub_imu;
LiveSdvioCore* globalLiveSdvio = NULL;

void image0CallBack(const sensor_msgs::ImageConstPtr &msg) 
{
    ros::Time t = msg->header.stamp;
//     ROS_INFO_STREAM("sdvio_node------image0CallBack: get image...");
    cv::Mat image  = cv_bridge::toCvShare(msg, std::string("mono8"))->image;
    globalLiveSdvio->mea_queue_mtx.lock();
//     globalLiveSdvio->image0_queue_mtx.lock();
    globalLiveSdvio->image0_buf.push_back(ImageMeasurement(t, image));
//     globalLiveSdvio->image0_queue_mtx.unlock();
    globalLiveSdvio->mea_queue_mtx.unlock();
    
    globalLiveSdvio->con.notify_one();
}

void image1CallBack(const sensor_msgs::ImageConstPtr &msg) 
{
    ros::Time t = msg->header.stamp;
//     ROS_INFO_STREAM("sdvio_node------image1CallBack: get image...");
    cv::Mat image  = cv_bridge::toCvShare(msg, std::string("mono8"))->image;
    globalLiveSdvio->mea_queue_mtx.lock();
//     globalLiveSdvio->image1_queue_mtx.lock();
    globalLiveSdvio->image1_buf.push_back(ImageMeasurement(t, image));
//     globalLiveSdvio->image1_queue_mtx.unlock();
    globalLiveSdvio->mea_queue_mtx.unlock();
    
    globalLiveSdvio->con.notify_one();
}

void imuCallBack(const sensor_msgs::ImuConstPtr &msg ) 
{
    ros::Time t = msg->header.stamp;
    Vector3d acc(msg->linear_acceleration.x,
		 msg->linear_acceleration.y,
		 msg->linear_acceleration.z);
    Vector3d ang(msg->angular_velocity.x,
		 msg->angular_velocity.y,
		 msg->angular_velocity.z);    
    globalLiveSdvio->mea_queue_mtx.lock();
//     globalLiveSdvio->imu_queue_mtx.lock();
    globalLiveSdvio->imu_buf.push_back(ImuMeasurement(t, acc, ang));
//     globalLiveSdvio->imu_queue_mtx.unlock();
    globalLiveSdvio->mea_queue_mtx.unlock();
    
    globalLiveSdvio->con.notify_one();
}

void process() 
{
    ROS_INFO("Thread starts...");
    
    globalLiveSdvio->Loop();
    
    ROS_INFO("Thread ends!");
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "sdvio_node");
    ros::NodeHandle pnh("~");
    
    readParameters(pnh);
    readIntrinsicParameter(CAM_NAMES[0]);

    sub_imu = pnh.subscribe(IMU_TOPIC, 1000, imuCallBack ) ;    
    sub_image[0] = pnh.subscribe(IMAGE_TOPIC_LEFT, 100, &image0CallBack );
//     if (NUM_OF_CAM == 2)
    sub_image[1] = pnh.subscribe(IMAGE_TOPIC_RIGHT, 100, &image1CallBack );


    LiveSdvioCore sdvio_node(pnh);
    globalLiveSdvio = &sdvio_node;
    boost::thread ptrProcessImageThread = boost::thread(&process);
    
    ros::spin();
    ptrProcessImageThread.join();
	
    return 0;
}