#pragma once

#include <global.h>
#include <performance_monitor.h>
#include <utility.h>
#include <cmath>


namespace sdvio
{
enum Method{GaussNewton, LevenbergMarquardt};

class SparseImgAlign
{
    static const int patch_halfsize = 2;
    static const int patch_size = 2 * patch_halfsize;
    static const int patch_area = patch_size * patch_size;
    static const int state_num = 6;
public:
    
    FramePtr 		cur_frame;
    FramePtr 		ref_frame;
    int 		level;
    int 		max_level;
    int 		min_level;
    bool 		have_ref_patch_cache;
    bool 		display;
    cv::Mat 		ref_patch_cache;
    cv::Mat 		resimg;
    std::vector<bool>   visible_fts;
    Eigen::Matrix<double, state_num, Eigen::Dynamic, Eigen::ColMajor> jacobian_cache;
    

    
    
    SparseImgAlign(      
	int _max_levels,
	int _min_level,
	int _n_iter,
	Method _method,
	bool _display,
	bool _verbose);
    ~SparseImgAlign();
    
    size_t align(FramePtr _cur_frame, FramePtr _ref_frame);
    
    bool optimize(SE3& _T_cur_ref);
    
    void reset();
    
    double computeResiduals(const SE3& _T_cur_ref, bool linearize_system, bool compute_weight_scale = false);
    
    void precomputeReferencePatches();
    
    int solve();
    
    void update (SE3& new_T);
    
    void startIteration();
    
    void finishIteration();    
    
    inline void applyPrior(const SE3& _T_cur_ref) 
    {
	Matrix3d R_cur_ref(_T_cur_ref.rotation_matrix());
	Vector3d ang_ypr(utility::R2ypr(R_cur_ref));
	
	x(0) = ang_ypr.x();
	x(1) = ang_ypr.y();
	x(2) = ang_ypr.z();	
	x(3) = _T_cur_ref.translation().x();
	x(4) = _T_cur_ref.translation().y();
	x(5) = _T_cur_ref.translation().z();
    }
protected:
    size_t					n_iter;
    size_t 					n_iter_init;
    size_t 					iter;
    size_t                			n_trials;
    size_t                			n_trials_max;          //!< Max number of trials
    size_t               			n_meas;                //!< Number of measurements
    Method 					method;
    double 					eps;
    double 					chi2;
    double 					rho;
    double                			mu_init, mu;
    double                			nu_init, nu;  
    bool 					have_prior;
    bool 					verbose;
    bool 					stop;
    bool            				use_weights;
    float               			scale;
    Matrix<double, state_num, state_num> 	H;
    Matrix<double, state_num, 1>		Jres;
    Matrix<double, state_num, 1>		x;		//!< update step
    Matrix<double, state_num, state_num>  	I_prior;
    
    
};
    
}