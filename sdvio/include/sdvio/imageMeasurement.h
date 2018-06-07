#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class ImageMeasurement 
{
public:
    ros::Time 			t;
    cv::Mat 			image;
    
    ImageMeasurement(const ros::Time& _t, const cv::Mat& _image):
	t(_t),
	image(_image)
    {}
    
    ImageMeasurement(const ImageMeasurement& _i)
    {
      t     = _i.t;
      image = _i.image.clone();
    }  
    
    ~ImageMeasurement() 
    {}
};