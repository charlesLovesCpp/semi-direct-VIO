#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

class ImuMeasurement 
{
public:
    ros::Time 			t;
    Eigen::Vector3d 		linear_acceleration;
    Eigen::Vector3d 		angular_velocity;

    ImuMeasurement(const ros::Time& _t, const Eigen::Vector3d& _acc, const Eigen::Vector3d& _ang):
	t(_t),
	linear_acceleration(_acc),
	angular_velocity(_ang)
    {}
    
    ImuMeasurement(const ros::Time& _t, const double& _acc_x, const double& _acc_y, const double& _acc_z,
					const double& _ang_x, const double& _ang_y, const double& _ang_z):
	t(_t)
    {
	linear_acceleration = Eigen::Vector3d(_acc_x, _acc_y, _acc_z);
	angular_velocity = Eigen::Vector3d(_ang_x, _ang_y, _ang_z);
    }    
    
    ImuMeasurement(const ImuMeasurement& _i) 
    {
      t = _i.t;
      linear_acceleration = _i.linear_acceleration;
      angular_velocity = _i.angular_velocity;
    }  
    
    ~ImuMeasurement() 
    {}
};