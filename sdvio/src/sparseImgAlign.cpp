#include <sparseImgAlign.h>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <utility.h>

namespace sdvio 
{
   
SparseImgAlign::SparseImgAlign(int _max_level, int _min_level, int _n_iter, Method _method, bool _display, bool _verbose)
{
    display = _display;
    max_level = _max_level;
    min_level = _min_level;    
    
    n_iter = _n_iter;
    n_iter_init = n_iter;
    method = _method;
    verbose = _verbose;
    eps = 0.000001;
}
 
SparseImgAlign::~SparseImgAlign()
{}
    
       
std::size_t SparseImgAlign::align(FramePtr _cur_frame, FramePtr _ref_frame)
{
    reset();
    
    if (_ref_frame->fts.empty()) {
	ROS_WARN_STREAM("SparseImgAlign: no features to track!");
	return 0;
    }
    
    cur_frame = _cur_frame;
    ref_frame = _ref_frame;
    
    // Create reference patch (row, column, data type)
    ref_patch_cache = cv::Mat(ref_frame->fts.size(), patch_area, CV_32F);
    
    // Determine the column of jacobian_cache by number of pixels
    jacobian_cache.resize(Eigen::NoChange, ref_patch_cache.rows * patch_area);
    
    
/*
    // Conducted outside
    if (cur_frame->isPreintegrated()) {
	Matrix3d R_cur_ref = cur_frame->R_w_b.inverse() * ref_frame->R_w_b;
    } else {
	cur_frame->T_w_b = ref_frame->T_w_b;
    }*/
    
    SE3 T_cur_ref(cur_frame->T_w_b.inverse() * ref_frame->T_w_b);
    
    for (level = max_level; level >= min_level; --level) {
	jacobian_cache.setZero();
	have_ref_patch_cache = false;	
	if(verbose)
	    printf("\nPYRAMID LEVEL %i\n---------------\n", level);
	bool res = false;
	res = optimize(T_cur_ref);
	if (!res) {
	    ROS_WARN_STREAM("SparseImgAlign: does not converge at pyramid " << level);
	    return 0;
	}    
    }
    
    cur_frame->T_w_b = ref_frame->T_w_b * T_cur_ref.inverse();
    
    return 1;
    
} 

bool SparseImgAlign::optimize(SE3& _T_cur_ref)
{
    SE3 T_cur_ref = _T_cur_ref;
    SE3 old_T_cur_ref;
    
    for (iter = 0; iter < n_iter; ++iter) {
	rho = 0;
	startIteration();
	
	H.setZero();
	Jres.setZero();
	
	// Compute initial error
	n_meas = 0;
	//  SE3& T_cur_from_ref, bool linearize_system, bool compute_weight_scale
	double new_chi2 = computeResiduals(T_cur_ref, true, false);
	
	if (have_prior)
	    applyPrior(T_cur_ref);
	
	if (!solve()) {
	    stop = true;
	}
	
	if (stop || (iter > 0 && new_chi2 > chi2)) {
	    if(verbose) {
		std::cout << "It. " << iter
			<< "\t Failure"
			<< "\t new_chi2 = " << new_chi2
			<< "\t Error increased. Stop optimizing."
			<< std::endl;
	    }
	    _T_cur_ref = old_T_cur_ref;
	    break;	    
	}
	
	// Update SE3
	old_T_cur_ref = T_cur_ref;
	update(T_cur_ref);
	
	chi2 = new_chi2;
	
	if(verbose) {
	    
	}	
	
	finishIteration();
	
	if(utility::norm_max(x)<=eps)
	    break;
    }
    return true;
}

int SparseImgAlign::solve()
{
    x = H.ldlt().solve(Jres);				//Solve it like solving Hx=Jres
    if((bool) std::isnan((double) x[0]))
	return 0;
    return 1;
}


void SparseImgAlign::update(SE3& new_T)
{
    new_T =  new_T * SE3::exp(-x);//Update estimated value.
}


double SparseImgAlign::computeResiduals(const SE3& _T_cur_ref, bool linearize_system, bool compute_weight_scale)
{
    const cv::Mat& cur_img = cur_frame->img_pyr[level];

    if (linearize_system && display)
	resimg = cv::Mat(cur_img.size(), CV_32F, cv::Scalar(0));
    
    if (have_ref_patch_cache == false)
	precomputeReferencePatches();

    // Compute the weights on the first iteration
    std::vector<float> errors;
    if(compute_weight_scale)
	errors.reserve(visible_fts.size());
  
    const int stride = cur_img.cols;
    const int border = patch_halfsize + 1;
    const float s = 1.0f / (1<<level);
    const Vector3d ref_pos(ref_frame->pos());
    float chi2 = 0.0;
    size_t feature_counter = 0; // is used to compute the index of the cached jacobian
    std::vector<bool>::iterator visiblity_it = visible_fts.begin();
    
    for (auto it = ref_frame->fts.begin(); it != ref_frame->fts.end(); ++it, ++feature_counter, ++visiblity_it) {
	if (!*visiblity_it)
	    continue;
	
	// Estimate features in cur_frame by _T_cur_ref
	const double depth = ((*it)->point->pos - ref_pos).norm();
	const Vector3d xyz_ref((*it)->f * depth);
	const Vector3d xyz_cur(_T_cur_ref * xyz_ref);
	const Vector2f uv_cur_pyr(cur_frame->w2c(xyz_cur).cast<float>() * s);
	const float u_cur = uv_cur_pyr[0];//column
	const float v_cur = uv_cur_pyr[1];//row, cv::Mat(row,column), Mat(v,u)
	const int u_cur_i = floorf(u_cur);
	const int v_cur_i = floorf(v_cur);
	
	// check if projection is within the image
	if(u_cur_i < 0 || v_cur_i < 0 || u_cur_i-border < 0 || v_cur_i-border < 0 || u_cur_i+border >= cur_img.cols || v_cur_i+border >= cur_img.rows)
	    continue;	
	
	// Compute bilateral interpolation weights for the current image
	const float subpix_u_cur = u_cur-u_cur_i;
	const float subpix_v_cur = v_cur-v_cur_i;
	const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
	const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
	const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
	const float w_cur_br = subpix_u_cur * subpix_v_cur;	
	
	// Corresponding patch of this feature
	float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache.data) + patch_area * feature_counter;
	size_t pixel_counter = 0; // is used to compute the index of the cached jacobian
	for (int y = 0; y < patch_size; ++y) {
	    uint8_t* cur_img_ptr = (uint8_t*) cur_img.data + (v_cur_i + y - patch_halfsize) * stride + (u_cur_i - patch_size);// charles I think it is u_cur_i * stride
	    for (int x = 0; x < patch_size; ++x) {
		const float intensity_cur = w_cur_tl * cur_img_ptr[0] + w_cur_tr * cur_img_ptr[1] + w_cur_bl * cur_img_ptr[stride] + w_cur_br * cur_img_ptr[stride + 1];
		const float res = intensity_cur - (*ref_patch_cache_ptr); // I(T*x) - T(x)
		
		if(compute_weight_scale)
		    errors.push_back(fabsf(res));	
		
		float weight = 1.0;
		if (use_weights) {
		    //TODO Compute weight from res
		}
		
		
		chi2 += res * res * weight;
		n_meas++;
		
		if (linearize_system) {
		    // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
		    const Vector6d J(jacobian_cache.col(feature_counter*patch_area + pixel_counter));
		    H.noalias() += J*J.transpose()*weight;
		    Jres.noalias() -= J*res*weight; // 因为这里的J是对residual的导数(如果只是对TemplatePatch的导数的话就-J),那么的话,我们要求的就是 -J*res,换算回来就成了 -= J*res
		    if(display)
			resimg.at<float>((int) v_cur+y-patch_halfsize, (int) u_cur+x-patch_halfsize) = res/255.0;		
		}		
	    }   
	}
	    
    }

    if (compute_weight_scale && iter == 0) {
	//TODO Compute the scale
    }
    
    // Return average error in pixels
    return chi2 / n_meas;
}

void SparseImgAlign::precomputeReferencePatches()
{
    const int border = patch_halfsize + 1;
    const cv::Mat& ref_img = ref_frame->img_pyr[level];
    const int stride = ref_img.cols;
    const float s = 1.0f / (1<<level);
    const Vector3d ref_pos = ref_frame->pos();
    const double focal_length = FOCAL_LENGTH;
    size_t feature_counter = 0;
    std::vector<bool>::iterator visiblity_it = visible_fts.begin();
    
    // Compute Jacobians for each patch pixel of their relative features
    for (auto it = ref_frame->fts.begin(), ite = ref_frame->fts.end(); it != ite; ++it, ++feature_counter, ++visiblity_it) {
	const float u_ref = (*it)->px[0] * s;
	const float v_ref = (*it)->px[1] * s;
	const int u_ref_i = floorf(u_ref);
	const int v_ref_i = floorf(v_ref);
	
	if ((*it)->point == NULL 
	    || u_ref_i - border < 0 		|| v_ref_i - border < 0 
	    || u_ref_i + border >= ref_img.cols || v_ref_i + border >= ref_img.rows)
	    continue;
	*visiblity_it = true;
	
	// cannot just take the 3d points coordinate because of the reprojection errors in the reference image!!!
	const double depth(((*it)->point->pos - ref_pos).norm());
	const Vector3d xyz_ref((*it)->f*depth);
    
	// evaluate projection jacobian 注意,这里的frame_jac是对residual的导数而不是直接对TemplatePatch的导数
	Matrix<double,2,6> frame_jac;
	Frame::jacobian_xyz2uv(xyz_ref, frame_jac);
    
	// compute bilateral interpolation weights for reference image
	const float subpix_u_ref = u_ref-u_ref_i;
	const float subpix_v_ref = v_ref-v_ref_i;
	const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
	const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
	const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
	const float w_ref_br = subpix_u_ref * subpix_v_ref;	
	
	size_t pixel_counter = 0;
	// Patch pixel intensity about to be filled
	float* cache_ptr = reinterpret_cast<float*> (ref_patch_cache.data) + patch_area * feature_counter;
	for (int y = 0; y < patch_size; ++y) {
	    uint8_t* ref_img_ptr = (uint8_t*) ref_img.data + (v_ref_i + y - patch_halfsize) * stride + (u_ref_i - patch_halfsize);
	    for (int x = 0; x < patch_size; ++x) {
		// Precompute interpolated reference patch color
		*cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] + w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride+1];
		//   [2]	dx = [1] - [0]
		// [0] [1]
		//   [3]	dy = [3] - [2]
		float dx = 0.5f * ((w_ref_tl*ref_img_ptr[1] + w_ref_tr*ref_img_ptr[2] + w_ref_bl*ref_img_ptr[stride+1] + w_ref_br*ref_img_ptr[stride+2])
				-(w_ref_tl*ref_img_ptr[-1] + w_ref_tr*ref_img_ptr[0] + w_ref_bl*ref_img_ptr[stride-1] + w_ref_br*ref_img_ptr[stride]));
		float dy = 0.5f * ((w_ref_tl*ref_img_ptr[stride] + w_ref_tr*ref_img_ptr[1+stride] + w_ref_bl*ref_img_ptr[stride*2] + w_ref_br*ref_img_ptr[stride*2+1])
				-(w_ref_tl*ref_img_ptr[-stride] + w_ref_tr*ref_img_ptr[1-stride] + w_ref_bl*ref_img_ptr[0] + w_ref_br*ref_img_ptr[1]));

		// Compute the jacobians
		jacobian_cache.col(patch_area * feature_counter + pixel_counter) =
		    (dx * frame_jac.row(0) + dy * frame_jac.row(1)) * (focal_length * s);
	    }
	}
    }
    have_ref_patch_cache = true;
}



void SparseImgAlign::finishIteration()
{

}

void SparseImgAlign::startIteration()
{

}

void SparseImgAlign::reset()
{
    have_prior = false;
    n_iter = n_iter_init;
    iter = 0;
    chi2 = 1e10;
    x.setZero();
}


}
