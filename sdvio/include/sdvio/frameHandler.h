#pragma once

#include <global.h>
#include <depthFilter.h>
#include <map.h>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <feature_manager.h>

namespace sdvio 
{

class Frame;

class FrameHandler
{
public:
    typedef list<Feature*> Features;	
	
    FramePtr			frames[WINDOW_SIZE];
    FramePtr 			frame_new;
    FramePtr 			frame_last;
    camodocal::CameraPtr*	cam;
    Map				map;
    DepthFilter* 		depth_filter; 
    int 			window_counter;
    bool 			is_initialized;
    vector<cv::Point2f>		fts_ref;
    vector<cv::Point2f>		fts_cur; 
    vector<FeatureManager*>	feature_managers;
    
    FrameHandler();
    ~FrameHandler();
    
    void reset();
    
    void start();
    
    void addFrame(Frame* _new_frame);
    
    FramePtr lastFrame() { return frame_last; }
    
    DepthFilter* depthFilter() const { return depth_filter; }
    
//     void slideWindow(Frame* frame, int marginalization_flag);
    
    bool trackFrame(Frame* _new_frame);
     
    void slideOldFrame();
    
    inline bool isInitialized() const { return is_initialized; };
    
};

}