#pragma once

#include <global.h>
#include <feature.h>
#include <feature_manager.h>

namespace sdvio 
{

class Frame;

class  Initializer
{
public:
    typedef list<Feature*> Features;
    
    FramePtr			frames[WINDOW_SIZE];
    int 			window_counter;
    int 			window_size;
    bool 			isInitialized;
    bool 			isReady;	
    Features			ref_fts;
    Features			cur_fts;
    vector<FeatureManager*>	feature_managers;
    
    // 现在初始化变成了直接从FrameHandler那里提取帧,然后在Initializer这里提取特征点,再进行初始化run,最后再把优化后的frames和三角化好的特征点返回回来,重新赋值给FrameHandler
    Initializer();
    ~Initializer();

    bool run(FramePtr (&_frames)[WINDOW_SIZE], vector<FeatureManager*> _feature_managers);

//     void reset();
//     
//     void resetDepth();
//     
//     void triangulateWindow();
//     
//     void addFrame(Frame* _new_frame);
//     
//     
//     
//     bool constructSFM();
//     
//     bool visualInitialAlign(); 
//     
//     bool solveGyroscopeBias(Vector3d& _delta_bg);
//     
//     bool linearAlignment(Vector3d& _g, VectorXd& _x);
//     
//     void RefineGravity(Vector3d& _g, VectorXd& _x);
//     
//     bool getGoodFramePair(Matrix3d& _R, Vector3d& _t, int& _l_idx, const int& _r_idx);
//     
//     void slideOldFrame();
//     
//     void slideSecondNewFrame();
//     
//     bool solvePoseByPnP(
// 	Frame* _frame,
// 	Vector3d& _P_initial,
// 	Matrix3d& _R_initial);  
//     
//     void triangulateTwoFrames(
// 	Frame* frame_i, 
// 	Eigen::Matrix<double, 3, 4> &Pose_i, 
// 	Frame* frame_j, 
// 	Eigen::Matrix<double, 3, 4> &Pose_j);    
//     
};
    

} // end sdvio