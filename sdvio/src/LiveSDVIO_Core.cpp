#include <LiveSDVIO_Core.h>
#include <initialization.h>

namespace sdvio {
    
LiveSdvioCore::LiveSdvioCore(ros::NodeHandle& pnh)
{
    last_acc.setZero();
    last_gyr.setZero();
    last_ba.setZero();
    last_bg.setZero();
}

LiveSdvioCore::~LiveSdvioCore()
{}

void LiveSdvioCore::Loop()
{  
    ROS_INFO_STREAM("LiveSdvioCore------Loop: starts...");
    while (true) {
	MeasurementPairs measurement_pairs;
	std::unique_lock<std::mutex> lk(mea_queue_mtx);
	con.wait(lk, [&] {
	    getMeasurementPairs(measurement_pairs);
	    return measurement_pairs.size() > 0;
	});
	lk.unlock();
	
// 	ROS_INFO_STREAM("LiveSdvioCore------Loop: get measurement_pairs...");
	for (auto& measurement_pair : measurement_pairs) {
	    
	    Frame* new_frame(new Frame(CAM_LEFT, measurement_pair.second.first, measurement_pair.first,
						   last_acc, last_gyr, last_ba, last_bg));
		
	    last_acc = measurement_pair.first.back().linear_acceleration;
	    last_gyr = measurement_pair.first.back().angular_velocity;

	    if (!frame_handler.isInitialized()) {
		// Add frame without features
		frame_handler.addFrame(new_frame);
// 		ROS_INFO_STREAM("LiveSdvioCore------Loop: window_counter:" << frame_handler.window_counter);
		
		if (frame_handler.window_counter == WINDOW_SIZE-1) {
		    frame_handler.slideOldFrame();
/*		    Initializer initializer;
		    initializer.run(frame_handler.frames, frame_handler.feature_managers);*/		    
		}
		    


		
		
// 		Initializer initializer(frame_handler.frames, frame_handler.feature_managers, WINDOW_SIZE);
// 		if (initializer.run())    
// 		    frame_handler.is_initialized = true;
		
		
		
	    } else {
		frame_handler.trackFrame(new_frame);
		last_ba = frame_handler.frames[frame_counter]->pose.Ba;
		last_bg = frame_handler.frames[frame_counter]->pose.Bg;		
	    }
	    

	}
    }
}

void LiveSdvioCore::getMeasurementPairs(MeasurementPairs& measurement_pairs)
{
//     ROS_INFO_STREAM("LiveSdvioCore------getMeasurementPairs: get measurement_pairs...");
    std::list<ImageMeasurement>::iterator iter0;
    std::list<ImageMeasurement>::iterator iter1;
    std::list<ImuMeasurement>::reverse_iterator riter_imu;
    std::list<ImuMeasurement>::iterator iter_imu;
    ros::Time tImage_start;
    cv::Mat image0;
    cv::Mat image1; 
    while (true)
    {
// 	ROS_INFO_STREAM("LiveSdvioCore------getMeasurementPairs: starts..." << image0_buf.size() << image0_buf.size() << imu_buf.size());	
        if (image0_buf.empty() || image1_buf.empty() || imu_buf.empty())
            return;

        iter0 = image0_buf.begin();
        iter1 = image1_buf.begin();	
	
        while ( iter1 != image1_buf.end() && iter0->t > iter1->t ){
            iter1 =  image1_buf.erase( iter1 ) ;
        }	
        while ( iter0 != image0_buf.end() && iter0->t < iter1->t ){
            iter0 =  image0_buf.erase( iter0 ) ;
        }	
	
        if ( iter1 == image1_buf.end() || iter0 == image0_buf.end() ) {
	    ROS_WARN("No synchronized images");
	    return;
	}
	    
	tImage_start = iter0->t;
	riter_imu = imu_buf.rbegin() ;
        if (riter_imu->t < tImage_start){
	    ROS_WARN("Wait for more imu measurements");
	    return;
        }	
	
	iter_imu = imu_buf.begin();
        if (iter_imu->t > tImage_start){
	    ROS_WARN("Imu measurements are too late");
	    iter0 =  image0_buf.erase( iter0 ) ;
	    iter1 =  image1_buf.erase( iter1 ) ;
	    continue;
        }	
	
        image0 = iter0->image.clone();
        image1 = iter1->image.clone();
	std::pair<cv::Mat, cv::Mat> image_pair = make_pair(image0, image1);
	iter0 =  image0_buf.erase( iter0 );
        iter1 =  image1_buf.erase( iter1 );
	
	std::vector<ImuMeasurement> IMUs;
	while (iter_imu->t < tImage_start) {
	    IMUs.emplace_back(*iter_imu);
	    iter_imu = imu_buf.erase(iter_imu);
	}
	
	if (iter_imu != imu_buf.end())
	    IMUs.emplace_back(*iter_imu);
	
        if (IMUs.empty()) {
	    ROS_WARN("No imu between two image");
	    continue;
	}

        measurement_pairs.emplace_back(IMUs, image_pair);
    }    
}


}