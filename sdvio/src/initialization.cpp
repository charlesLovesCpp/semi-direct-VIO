#include <initialization.h>
#include <utility.h>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <feature_detector.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>

#include <initial_factor.h>

namespace sdvio 
{ 
  
Initializer::Initializer():
    window_counter(0),
    isInitialized(false),
    isReady(false)
{
}
    
    
Initializer::~Initializer()
{}


bool Initializer::run(FramePtr (&_frames)[WINDOW_SIZE], std::vector< FeatureManager* > _feature_managers)
{
    for (int i = 0; i < WINDOW_SIZE; ++i) {
	ROS_INFO_STREAM("Initializer------run: "<< _frames[i]->id << " Feature number: " << _frames[i]->fts.size());
    }
    
    
//     _frames[WINDOW_SIZE-1]->id
    
//     if(!constructSFM()) {
// 	ROS_WARN_STREAM("Initialization: fail to construct SFM");
// 	return false;
//     }
// 
//     if(!visualInitialAlign()) {
// 	ROS_WARN_STREAM("Initialization: fail to align camera and IMU");
// 	return false;
//     }
    
    
    isInitialized = true;
    return true;
}


// bool Initializer::run(FramePtr* _frames, std::vector< FeatureManager* > _feature_managers)
// {
//     ROS_INFO_STREAM("Initializer------run:");
//     
//     
// //     if(!constructSFM()) {
// // 	ROS_WARN_STREAM("Initialization: fail to construct SFM");
// // 	return false;
// //     }
// // 
// //     if(!visualInitialAlign()) {
// // 	ROS_WARN_STREAM("Initialization: fail to align camera and IMU");
// // 	return false;
// //     }
//     
//     
//     isInitialized = true;
//     return true;
// }
 
/* 
bool Initializer::solvePoseByPnP(
    Frame* _frame,
    Vector3d& _P_initial,
    Matrix3d& _R_initial)
{
    vector<cv::Point3f> pts;
    vector<cv::Point2f> pxs;
    
    for (auto ftr : _frame->fts) {
	FeatureManager* ftr_manager = dynamic_cast<FeatureManager*>(ftr->point);
	if (ftr_manager->triang_type != FeatureManager::TYPE_TRIANGULATED)
	    continue;
	Vector3d pt(ftr->point->getPos());
	Vector2d px(ftr->getPtc());
	pts.push_back(cv::Point3d(pt.x(), pt.y(), pt.z()));
	pxs.push_back(cv::Point2d(px.x(), px.y()));
    }    
    if (int(pxs.size()) < 15) {
	if (int(pxs.size()) < 10)
	    return false;
    }
    
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(_R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(_P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts, pxs, K, D, rvec, t, 1);
    if(!pnp_succ) {
	return false;
    }
    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    _R_initial = R_pnp;
    _P_initial = T_pnp;
    return true;    
}


void Initializer::triangulateTwoFrames(
    Frame* frame_i, 
    Eigen::Matrix<double, 3, 4> &Pose_i, 
    Frame* frame_j, 
    Eigen::Matrix<double, 3, 4> &Pose_j)
{
    assert(frame_i != frame_j);
    vector<std::pair<Feature*, Feature*>> matches;
    utility::getCorrespondingPoints(frame_i, frame_j, matches);
    
    for (auto match : matches) {
	Vector3d pos;
	utility::triangulatePoint(Pose_i, Pose_j, match.first->f, match.second->f, pos);	
	FeatureManager* ftr_manager = dynamic_cast<FeatureManager*>(match.first->point);
	ftr_manager->triang_type = FeatureManager::TYPE_TRIANGULATED;
	ftr_manager->setPos(pos);
    }
}


void Initializer::addFrame(Frame* _new_frame)
{  
    frames[window_counter].reset(_new_frame);
    frames[window_counter]->setWindowCounter(window_counter);
   
    //TODO detect features
    feature_detection::FastDetector detector(
	frames[window_counter]->img().cols, frames[window_counter]->img().rows, GRID_SIZE, NUM_PYR_LEVELS);
    
    // Track old features
    if (window_counter > 0 && !ref_fts.empty()) {
	Features tracked_features;
	utility::trackFeatures(frames[window_counter-1].get(), frames[window_counter].get(), ref_fts, tracked_features);
	detector.setExistingFeatures(tracked_features);
	cur_fts.insert(cur_fts.end(), tracked_features.begin(), tracked_features.end());

	// Add observation into existing feature_manager
	auto it_ref = ref_fts.begin();
	auto it_tracked = tracked_features.begin();
	for (; it_ref != ref_fts.end(); ++it_ref, ++it_tracked) {   
	    (*it_ref)->point->addFrameRef(*it_tracked);
	    (*it_tracked)->point = (*it_ref)->point;    
	}
    }
    
    // Detect new features
    Features new_features;
    detector.detect(frames[window_counter].get(), frames[window_counter]->img_pyr, TRIANG_MIN_CORNER_SCORE , new_features); 
    cur_fts.insert(cur_fts.end(), new_features.begin(), new_features.end());

    // Create new feature_manager for new feature
    for (auto it = new_features.begin(); it != new_features.end(); ++it) {
	frames[window_counter]->addFeature(*it);
	FeatureManager* ftr_manager_ptr = new FeatureManager(*it);
	(*it)->point = ftr_manager_ptr;
	feature_managers.push_back(ftr_manager_ptr);
    }
    
    // Add all features with id in current frame
    frames[window_counter]->fts.clear();
    std::for_each(cur_fts.begin(), cur_fts.end(), [&](Feature* ftr) {
	frames[window_counter]->fts.push_back(ftr);
    });
    
    if (window_counter < WINDOW_SIZE-1)
	window_counter++;
    else
	isReady = true;
    
    // If window is full, slide out the oldest frame.
    if (window_counter == WINDOW_SIZE-1) {
	double parallex;
	bool res = utility::computeParallex(frames[window_counter-1].get(), frames[window_counter].get(), parallex);
	if (parallex > MIN_PARALLAX || res == false)
	    slideOldFrame();
	else
	    slideSecondNewFrame();
    }

    ref_fts = cur_fts;
    cur_fts.clear();
}
   
void Initializer::slideOldFrame()
{
    for (int i = 0; i < WINDOW_SIZE-1; ++i) {
	frames[i].swap(frames[i+1]);
	frames[i]->setWindowCounter(i);
    }
    frames[WINDOW_SIZE-1]->clear();
}

void Initializer::slideSecondNewFrame()
{
    frames[WINDOW_SIZE-2].swap(frames[WINDOW_SIZE-1]);
    frames[WINDOW_SIZE-2]->setWindowCounter(WINDOW_SIZE-2);
    frames[WINDOW_SIZE-1]->clear();
}



bool Initializer::linearAlignment(Vector3d& _g, VectorXd& _x)
{
    VectorXd x;
    x.setZero();
    
    int n_state = WINDOW_SIZE * 3 + 3 + 1;
    Eigen::MatrixXd A(n_state, n_state);
    VectorXd b(n_state);
    A.setZero();
    b.setZero();
    for (unsigned int i = 0; i < WINDOW_SIZE-1; ++i) {
	Eigen::MatrixXd H(6, 10);
	VectorXd z(6);
	H.setZero();
	z.setZero();
	
	double dt = frames[i+1]->preintegration->sum_dt;

        H.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        H.block<3, 3>(0, 6) = frames[i]->pose.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
	H.block<3, 1>(0, 9) = frames[i]->pose.R.transpose() * (frames[i]->pose.P - frames[i+1]->pose.P) / 100.0; // Why 100?    
        z.block<3, 1>(0, 0) = frames[i+1]->preintegration->delta_p + frames[i]->pose.R.transpose() * frames[i+1]->pose.R * TIC[0] - TIC[0];
	
        H.block<3, 3>(3, 0) = -Matrix3d::Identity();
        H.block<3, 3>(3, 3) = frames[i]->pose.R.transpose() * frames[i+1]->pose.R;
        H.block<3, 3>(3, 6) = frames[i]->pose.R.transpose() * dt * Matrix3d::Identity();
        z.block<3, 1>(3, 0) = frames[i+1]->preintegration->delta_v;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        cov_inv.setIdentity();
	
	MatrixXd tmp_A = H.transpose() * cov_inv * H;
	VectorXd tmp_b = H.transpose() * cov_inv * z;
	
	A.block<6, 6>(i*3, i*3) += tmp_A.topLeftCorner<6, 6>();
	b.segment<6>(i*3) += tmp_b.head<6>();
	
	A.bottomRightCorner<4, 4>() += tmp_A.bottomRightCorner<4, 4>();
	b.tail<4>() += tmp_b.tail<4>();
	
	A.block<6, 4>(i*3, n_state - 4) += tmp_A.topRightCorner<6, 4>();
	A.block<4, 6>(n_state - 4, i*3) += tmp_A.bottomLeftCorner<4, 6>();
    }
  
    A = A * 1000;
    b = b * 1000;
    x = A.ldlt().solve(b);
    double scale = x(n_state - 1) / 100.0;
    (x.tail<1>())(0) = scale;
    Vector3d g_hat = x.segment<3>(n_state - 4);
    if (std::isnan(g_hat(0)) || std::isnan(g_hat(1)) || std::isnan(g_hat(2))) {
	ROS_WARN("Initialization: values in g_hat is nan");
	return false;
    } 
    
    if(fabs(g_hat.norm() - G.norm()) > 1.0) {
	ROS_WARN("Initialization: g_hat does not converge");
	return false;
    } 
    
    ROS_WARN_STREAM("Initialization: estimated gravity: " << g_hat.norm() << " " << g_hat.transpose());    
    
    RefineGravity(g_hat, x);
    scale = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = scale; 
    if(scale < 0.0 ) {
	ROS_WARN_STREAM("Initialization: scale < 0");
	return false;  
    }
    
    ROS_WARN_STREAM("Initialization: refined gravity: " << g_hat.norm() << " " << g_hat.transpose());

    _x = x;
    _g = g_hat;
    
    return true; 
}

MatrixXd TangentBasis(Vector3d& _g)
{
    Vector3d b, c;
    Vector3d a = _g.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)// if _g does point to z axis
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void Initializer::RefineGravity(Vector3d& _g, VectorXd& _x)
{
    Vector3d g0 = _g.normalized() * G.norm();
    Vector3d lx, ly;
    int n_state = WINDOW_SIZE * 3 + 2 + 1;
      
    Eigen::MatrixXd A(n_state, n_state);
    VectorXd b(n_state);
    A.setZero();
    b.setZero();

    for (int k = 0; k < 4; ++k) {
	MatrixXd lxly(3, 2);
	lxly = TangentBasis(g0);
	
	for (unsigned int i = 0; i < WINDOW_SIZE; ++i) {
	    Eigen::MatrixXd H(6, 9);
	    VectorXd z(6);
	    H.setZero();
	    z.setZero();

	    double dt = frames[i+1]->preintegration->sum_dt;

	    H.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
	    H.block<3, 2>(0, 6) = frames[i]->pose.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
	    H.block<3, 1>(0, 8) = frames[i]->pose.R.transpose() * (frames[i+1]->pose.P - frames[i]->pose.P) / 100.0;
	    z.block<3, 1>(0, 0) = frames[i+1]->preintegration->delta_p + frames[i]->pose.R.transpose() * frames[i+1]->pose.R * TIC[0] - TIC[0] - frames[i]->pose.R.transpose() * dt * dt / 2 * g0;

	    H.block<3, 3>(3, 0) = -Matrix3d::Identity();
	    H.block<3, 3>(3, 3) = frames[i]->pose.R.transpose() * frames[i+1]->pose.R;
	    H.block<3, 2>(3, 6) = frames[i]->pose.R.transpose() * dt * Matrix3d::Identity() * lxly;
	    z.block<3, 1>(3, 0) = frames[i+1]->preintegration->delta_v - frames[i]->pose.R.transpose() * dt * Matrix3d::Identity() * g0;

	    Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
	    cov_inv.setIdentity();
	    
	    MatrixXd tmp_A = H.transpose() * cov_inv * H;
	    VectorXd tmp_b = H.transpose() * cov_inv * z;

	    A.block<6, 6>(i*3, i*3) += tmp_A.topLeftCorner<6, 6>();
	    b.segment<6>(i*3) += tmp_b.head<6>();
	    
	    A.bottomRightCorner<3, 3>() += tmp_A.bottomRightCorner<3, 3>();
	    b.tail<3>() += tmp_b.tail<3>();

	    A.block<6, 3>(i*3, n_state - 3) += tmp_A.topRightCorner<6, 3>();
	    A.block<3, 6>(n_state - 3, i*3) += tmp_A.bottomLeftCorner<3, 6>();	    
	    
	}

	A = A * 1000.0;
	b = b * 1000.0;
	_x = A.ldlt().solve(b);

	VectorXd dg = _x.segment<2>(n_state - 3);
	g0 = (g0 + lxly * dg).normalized() * G.norm();	
    } 
    _g = g0;    
}


bool Initializer::solveGyroscopeBias(Vector3d& _delta_bg)
{
    Eigen::Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    delta_bg.setZero();
    A.setZero();
    b.setZero();
    
    for (unsigned int i = 0; i < WINDOW_SIZE-1; ++i) {
	Eigen::Matrix3d J_bg;
	J_bg.setZero();
	J_bg = frames[i]->preintegration->jacobian.template block<3, 3>(O_R, O_BG);
	
	Vector3d residual;
	residual.setZero();
	Eigen::Quaterniond q_ij(frames[i]->pose.R.transpose() * frames[i+1]->pose.R); 
	residual = 2 * (frames[i]->preintegration->delta_q.inverse() * q_ij).vec();
	
	A += J_bg.transpose() * J_bg;
	b += J_bg.transpose() * residual;
    }
    A = A * 1000;
    b = b * 1000;
    delta_bg = A.ldlt().solve(b);
    
    if (std::isnan(delta_bg(0)) || std::isnan(delta_bg(1)) || std::isnan(delta_bg(2))) {
	ROS_WARN("Initialization: gyroscope bias is nan");
	return false;
    }     
    
    ROS_WARN_STREAM("Initialization: gyroscope bias initial calibration: " << delta_bg.transpose());
    
    _delta_bg = delta_bg;
    
    return true;
}


bool Initializer::visualInitialAlign()
{
    Vector3d delta_bg;
    if(!solveGyroscopeBias(delta_bg))
	return false;
    
    // Reset bias
    for (unsigned int i = 0; i <= WINDOW_SIZE-1; ++i) {
	frames[i]->pose.setBa(Vector3d::Zero());
	frames[i]->pose.setBg(delta_bg);
	frames[i]->preintegration->repropagate(Vector3d::Zero(), frames[i]->pose.Bg);
    }
    
    Vector3d g; VectorXd x;
    if(!linearAlignment(g, x))
        return false;

    // Reset gravity vector
    G_w = g;
    SCALE = (x.tail<1>())(0);
    
    // Reset scaled positions and velocitie
    Vector3d P_0 = SCALE * frames[0]->pose.P - frames[0]->pose.R * TIC[0];
    for (int i = 0; i < WINDOW_SIZE; ++i) {
	Vector3d P_w_c = frames[i]->pose.P;
	Matrix3d R_w_b = frames[i]->pose.R;
	// P_w_b = P_w_c + R_w_b * t_b_c
	frames[i]->pose.setPos(SCALE * P_w_c - R_w_b * TIC[0] - P_0);
	frames[i]->pose.setVel(R_w_b * x.segment<3>(i*3));
    }
  
    // Reset position and rotation
    resetDepth();
    triangulateWindow();
    
    for (auto it = feature_managers.begin(); it != feature_managers.end(); ++it) {
	if ((*it)->triang_type != FeatureManager::TYPE_TRIANGULATED) {
	    for (auto it_ftr = (*it)->obs.begin(); it_ftr != (*it)->obs.end(); ++it_ftr) {
		(*it_ftr)->frame->deleteFeature(*it);
	    }
	}
	it = feature_managers.erase(it);
    }
    
    
    return true;
}

void Initializer::triangulateWindow()
{  
    for (auto& ftr_manager : feature_managers) {
	if (!(ftr_manager->obs.size() >= 2 && ftr_manager->obs.front()->frame->window_counter < WINDOW_SIZE - 2))
	    continue;
	
	if (ftr_manager->triang_type == FeatureManager::TYPE_TRIANGULATED)
	    continue;
	
	ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * (ftr_manager->obs.size()-1), 4);
	
        int svd_idx = 0;
	Vector3d P0 = frames[ftr_manager->obs.front()->frame->window_counter]->pose.P;
	Matrix3d R0 = frames[ftr_manager->obs.front()->frame->window_counter]->pose.R;
	ftr_manager->obs.front()->frame->b2c(P0, R0);

	for (auto ftr : ftr_manager->obs) {
	    
	    if (ftr == ftr_manager->obs.front())
		continue;
	    
	    Vector3d P1 = frames[ftr->frame->window_counter]->pose.P;
	    Matrix3d R1 = frames[ftr->frame->window_counter]->pose.R;
	    ftr->frame->b2c(P1, R1);	    
	    
            Vector3d t = R0.transpose() * (P1 - P0);// relative translation represented in t0 frame. t_0_1
            Matrix3d R = R0.transpose() * R1;// relative rotation. R_w_w * R_W_b = R_w_b, R_0_1
            Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();// We want R_1_0, so that we can compute points represented in F0
            P.rightCols<1>() = -R.transpose() * t;// t_1_0
            Vector3d f = ftr->f;
	    
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);	
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);	    
	}
	
	ROS_ASSERT(svd_idx == svd_A.rows());
	Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
	Vector3d xyz_c(svd_V[0]/svd_V[3], svd_V[1]/svd_V[3], svd_V[2]/svd_V[3]);
	
	if (xyz_c.z() < 0.1)
	    continue;	
	
	Vector3d xyz_w = ftr_manager->obs.front()->frame->f2w(xyz_c);
	ftr_manager->pos = xyz_w;
	ftr_manager->triang_type = FeatureManager::TYPE_TRIANGULATED;
	
	// 1. Convert pose from F_w_b to F_w_c
	// 2. Compute 3D point in Fc_start
	// 3. Convert point from Fc_start to Fb_start
	// 4. Convert point from Fb_start to Fb0
	// 5. Assign 3D point for ftr_manager, and set flag
    }
}

void Initializer::resetDepth()
{
    std::for_each(feature_managers.begin(), feature_managers.end(), [&](FeatureManager* ftr_manager){
	ftr_manager->triang_type = FeatureManager::TYPE_INITIAL;
    });
}

bool Initializer::constructSFM()
{
    resetDepth();
    Matrix3d relative_R;
    Vector3d relative_T;
    int l_idx;
    int r_idx = WINDOW_SIZE-1;
    if (!getGoodFramePair(relative_R, relative_T, l_idx, r_idx)) {
	ROS_WARN_STREAM("Initialization------constructSFM: fail for no enough motion");
	return false;
    }

    Matrix3d c_Rotation[WINDOW_SIZE];
    Vector3d c_Translation[WINDOW_SIZE];
    Quaterniond c_Quat[WINDOW_SIZE];
    double c_rotation[WINDOW_SIZE][4];
    double c_translation[WINDOW_SIZE][3];
    Eigen::Matrix<double, 3, 4> Pose[WINDOW_SIZE];
    
    // Set the l^th frame as the origin
    Matrix3d R_init; R_init.setIdentity();
    frames[l_idx]->pose.setPos(Vector3d(0, 0, 0));
    frames[l_idx]->pose.setRot(R_init);
    frames[r_idx]->pose.setPos(relative_T);
    frames[r_idx]->pose.setRot(frames[l_idx]->pose.R * Quaterniond(relative_R));    
    
    
    c_Quat[l_idx] = frames[l_idx]->pose.Q.inverse();
    c_Rotation[l_idx] = c_Quat[l_idx].toRotationMatrix();
    c_Translation[l_idx] = -1 * (c_Rotation[l_idx] * frames[l_idx]->pose.P);// p_c = R_t_w * p_w + t_c, t_c = - R^-1 * t_w
    Pose[l_idx].block<3, 3>(0, 0) = c_Rotation[l_idx];
    Pose[l_idx].block<3, 1>(0, 3) = c_Translation[l_idx];    
    
    // do the same things to the newest frame
    c_Quat[WINDOW_SIZE - 1] = frames[r_idx]->pose.Q.inverse();
    c_Rotation[WINDOW_SIZE - 1] = c_Quat[WINDOW_SIZE - 1].toRotationMatrix();
    c_Translation[WINDOW_SIZE - 1] = -1 * (c_Rotation[WINDOW_SIZE - 1] * frames[r_idx]->pose.P);
    Pose[WINDOW_SIZE - 1].block<3, 3>(0, 0) = c_Rotation[WINDOW_SIZE - 1];
    Pose[WINDOW_SIZE - 1].block<3, 1>(0, 3) = c_Translation[WINDOW_SIZE - 1];    
    
    //1: trangulate between l ----- frame_num - 1
    //2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1; 
    for (int i = l_idx; i < WINDOW_SIZE - 1 ; i++) {
	if (i > l_idx) {
	    Matrix3d R_initial = c_Rotation[i - 1];// at first, this is c_Rotation[l]
	    Vector3d P_initial = c_Translation[i - 1];
	    if(!solvePoseByPnP(frames[i].get(), P_initial, R_initial))// it inputs all sfm measurements
		return false;
	    c_Rotation[i] = R_initial;// Output pose of next frame
	    c_Translation[i] = P_initial;
	    c_Quat[i] = c_Rotation[i];
	    Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
	    Pose[i].block<3, 1>(0, 3) = c_Translation[i];
	}

	// triangulate point based on the solve pnp result
	triangulateTwoFrames(frames[i].get(), Pose[i], frames[WINDOW_SIZE-1].get(), Pose[WINDOW_SIZE - 1]);
    }    
    //3: triangulate l-----l+1 l+2 ... frame_num -2
    for (int i = l_idx + 1; i < WINDOW_SIZE - 1; i++)
	triangulateTwoFrames(frames[l_idx].get(), Pose[l_idx], frames[i].get(), Pose[i]);    
    //4: solve pnp l-1; triangulate l-1 ----- l
    //             l-2              l-2	 ----- l
    for (int i = l_idx - 1; i >= 0; i--)
    {
	//solve pnp
	Matrix3d R_initial = c_Rotation[i + 1];
	Vector3d P_initial = c_Translation[i + 1];
	if(!solvePoseByPnP(frames[i].get(), P_initial, R_initial))
		return false;
	c_Rotation[i] = R_initial;
	c_Translation[i] = P_initial;
	c_Quat[i] = c_Rotation[i];
	Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
	Pose[i].block<3, 1>(0, 3) = c_Translation[i];
	//triangulate
	triangulateTwoFrames(frames[i].get(), Pose[i], frames[l_idx].get(), Pose[l_idx]);
    }
    //5: triangulate all other points
    std::for_each(feature_managers.begin(), feature_managers.end(), [&](FeatureManager* ftr_manager){
	if (ftr_manager->triang_type == FeatureManager::TYPE_TRIANGULATED)
	    return;
	if (ftr_manager->obs.size() >= 2) {
	    int index_first = ftr_manager->obs.front()->frame->window_counter;
	    int index_last = ftr_manager->obs.back()->frame->window_counter;
	    Vector3d pos;
	    utility::triangulatePoint(
		Pose[index_first], 
		Pose[index_last], 
		ftr_manager->obs.front()->f, 
		ftr_manager->obs.back()->f, 
		pos);	
	    ftr_manager->triang_type = FeatureManager::TYPE_TRIANGULATED;
	    ftr_manager->setPos(pos);	
	}
    });  
    
    //full BA
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    for (int i = 0; i < WINDOW_SIZE-1; ++i) {
	c_translation[i][0] = c_Translation[i].x();
	c_translation[i][1] = c_Translation[i].y();
	c_translation[i][2] = c_Translation[i].z();
	c_rotation[i][0] = c_Quat[i].w();
	c_rotation[i][1] = c_Quat[i].x();
	c_rotation[i][2] = c_Quat[i].y();
	c_rotation[i][3] = c_Quat[i].z();
	problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
	problem.AddParameterBlock(c_translation[i], 3);
	if (i == l_idx) {
	    problem.SetParameterBlockConstant(c_rotation[i]);
	}
	if (i == l_idx || i == WINDOW_SIZE - 1) {
	    problem.SetParameterBlockConstant(c_translation[i]);
	}
    }    
    
    for (auto it = feature_managers.begin(); it != feature_managers.end(); ++it) {
	if ((*it)->triang_type != FeatureManager::TYPE_TRIANGULATED)
	    continue;
	auto it_ftr = (*it)->obs.begin();
	auto it_end = (*it)->obs.end();
	for (;it_ftr != it_end; ++it_ftr) {
	    int idx = (*it_ftr)->frame->window_counter;
	    Vector2d pt_c = (*it_ftr)->getPtc();
	    ceres::CostFunction* cost_function = ReprojectionError3D::Create(pt_c.x(), pt_c.y());
	    problem.AddResidualBlock(cost_function, NULL, c_rotation[idx], c_translation[idx], (*it)->position);
	}
    }  
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03) {
	    
    }
    else {
	ROS_WARN_STREAM("Initialization------constructSFM: ceres does not converge.");
	return false;
    }    
    
    for (int i = 0; i < WINDOW_SIZE-1; ++i) {
	Quaterniond q;
	Vector3d p;
	q.w() = c_rotation[i][0]; 
	q.x() = c_rotation[i][1]; 
	q.y() = c_rotation[i][2]; 
	q.z() = c_rotation[i][3]; 
	q = q.inverse();
	p = -1 * (q * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
	frames[i]->pose.setPos(p); 						// p_w_ck in paper
	frames[i]->pose.setRot(q.toRotationMatrix() * RIC[0].transpose()); 	// R_w_bk in paper
	frames[i]->setKeyframe();
    }
    
    for (auto it = feature_managers.begin(); it != feature_managers.end(); ++it) {
	if ((*it)->triang_type != FeatureManager::TYPE_TRIANGULATED)
	    continue;
	(*it)->update();
    }     
    
    return true;
    
}

bool Initializer::getGoodFramePair(Matrix3d& _R, Vector3d& _t, int& _l_idx, const int& _r_idx)
{	
    for (int i = 0; i < _r_idx; ++i) {
	double parallex;
	bool res = utility::computeParallex(frames[i].get(), frames[_r_idx].get(), parallex);
	if (res && (parallex > MIN_PARALLAX)) {
	    _l_idx = i;
	    return true;
	}
    }
    return false;
}


void Initializer::reset()
{
    window_counter = 0;
    isInitialized = false;
    isReady = false;
    std::for_each(frames, frames+WINDOW_SIZE, [&](FramePtr frame){
	frame.reset();
    });
}*/
    

} // end sdvio