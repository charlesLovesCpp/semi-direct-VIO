#pragma once

#include <global.h>
#include <boost/noncopyable.hpp>
#include <integration_base.h>
#include <imuMeasurement.h>
#include <utility.h>

namespace sdvio
{

class Point;
class Feature;

typedef list<Feature*> Features;
typedef vector<cv::Mat> ImgPyr;

// typedef vector<pair<vector<ImuMeasurement>, pair<cv::Mat, cv::Mat>>> MeasurementPairs;

class Pose 
{
public:
    Eigen::Vector3d 		P;	//!< P_w
    Eigen::Matrix3d		R;	//!< R_w_b
    Eigen::Quaterniond 		Q;      //!< Q_w_b
    Eigen::Vector3d 		V;
    Eigen::Vector3d 		Ba;
    Eigen::Vector3d 		Bg;
    
    double para_Pose[SIZE_POSE];
    double para_SpeedBias[SIZE_SPEEDBIAS];
    
    Pose() {
	R.setIdentity();
	Q = R;
	P.setZero();
	V.setZero();
	Ba.setZero();
	Bg.setZero();
	
	para_Pose[0] = P.x();
	para_Pose[1] = P.y();
	para_Pose[2] = P.z();
	para_Pose[3] = Q.x();
	para_Pose[4] = Q.y();
	para_Pose[5] = Q.z();
	para_Pose[6] = Q.w();

	para_SpeedBias[0] = V.x();
	para_SpeedBias[1] = V.y();
	para_SpeedBias[2] = V.z();   
	para_SpeedBias[3] = Ba.x();
	para_SpeedBias[4] = Ba.y();
	para_SpeedBias[5] = Ba.z();  
	para_SpeedBias[6] = Bg.x();
	para_SpeedBias[7] = Bg.y();
	para_SpeedBias[8] = Bg.z();
    };
    ~Pose() {};
    
    inline void setPos(const Vector3d& _P) {
	P = _P;
	para_Pose[0] = P.x();
	para_Pose[1] = P.y();
	para_Pose[2] = P.z();	
    };

    inline void setRot(const Matrix3d& _R) {
	R = _R;
	Q = R;
	para_Pose[3] = Q.x();
	para_Pose[4] = Q.y();
	para_Pose[5] = Q.z();
	para_Pose[6] = Q.w();
    };
    
    inline void setVel(const Vector3d& _V) {
	V = _V;
	para_SpeedBias[0] = V.x();
	para_SpeedBias[1] = V.y();
	para_SpeedBias[2] = V.z();
    
    };
    
    inline void setBa(const Vector3d& _Ba) {
	Ba = _Ba;
	para_SpeedBias[3] = Ba.x();
	para_SpeedBias[4] = Ba.y();
	para_SpeedBias[5] = Ba.z();
    };
    
    inline void setBg(const Vector3d& _Bg) {
	Bg = _Bg;
	para_SpeedBias[6] = Bg.x();
	para_SpeedBias[7] = Bg.y();
	para_SpeedBias[8] = Bg.z();
    };  
    
    inline void updatePos() {
	P = Vector3d(para_Pose[0], para_Pose[1], para_Pose[2]);
    };
    
    inline void updateRot() {
	Q = Eigen::Quaterniond(para_Pose[6], para_Pose[3], para_Pose[4], para_Pose[5]);  
	R = Q.toRotationMatrix();	
    };
    
    inline void updateVel() {
	V = Vector3d(para_SpeedBias[0], para_SpeedBias[1], para_SpeedBias[2]);
    };
    
    inline void updateBa() {
	Ba = Vector3d(para_SpeedBias[3], para_SpeedBias[4], para_SpeedBias[5]);
    };
    
    inline void updateBg() {
	Bg = Vector3d(para_SpeedBias[6], para_SpeedBias[7], para_SpeedBias[8]);
    }; 
};


class Frame : boost::noncopyable
{
public:
    static int			frame_counter;
    int				id;
    int 			window_counter;
    double 			timestamp;
    camodocal::CameraPtr	cam;
    camodocal::CameraPtr	cam1;
    Eigen::Matrix<double, 6, 6> Cov; 
    ImgPyr			img_pyr;
    ImgPyr			img_pyr1;
    Features                    fts;
    bool 			is_keyframe;
    bool 			is_preintegrated;
    IntegrationBase*		preintegration;
    Sophus::SE3			T_w_b;
    Pose 			pose;
    
    
    Frame(camodocal::CameraPtr cam, const cv::Mat& img, double timestamp);
    Frame(camodocal::CameraPtr cam0, const cv::Mat& _img0, camodocal::CameraPtr cam1, const cv::Mat& _img1, const vector<ImuMeasurement>& _imus, 
	    const Eigen::Vector3d& _acc0, const Eigen::Vector3d& _gyr0,
	    const Eigen::Vector3d& _ba0, const Eigen::Vector3d& _bg0);  
    Frame(camodocal::CameraPtr cam, const cv::Mat& _img, const vector<ImuMeasurement>& _imus, 
	    const Eigen::Vector3d& _acc0, const Eigen::Vector3d& _gyr0,
	    const Eigen::Vector3d& _ba0, const Eigen::Vector3d& _bg0);
    
    ~Frame();
    
    
    /// Clear all features 
    void clear();
    
    /// Add a feature to the image
    void addFeature(Feature* ftr);
    
    /// Delete a feature in obs
    bool deleteFeature(Point* point);
    
    /// Select this frame as keyframe.
    void setKeyframe();
    
    /// Initialize new frame and create image pyramid.
    void initFrame(const cv::Mat& _img);
    
    /// Initialize new frame and create image pyramid.
    void initFrameStereo(const cv::Mat& _img0, const cv::Mat& _img1);
    
    /// Initialize IMU preintegration.
    void initIMU(const vector<ImuMeasurement>& _imus, const Eigen::Vector3d& _acc0, const Eigen::Vector3d& _gyr0, const Eigen::Vector3d& _ba0, const Eigen::Vector3d& _bg0);    
    
    void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

    bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min); 
    
    /// Full resolution image stored in the frame.
    inline const cv::Mat& img() const { return img_pyr[0]; }

    /// Was this frame selected as keyframe?
    inline bool isKeyframe() const { return is_keyframe; }
    
    inline void setWindowCounter(int _window_counter) { window_counter = _window_counter; } ;

    /// Was this frame preintegrated?
    inline bool isPreintegrated() const { return is_preintegrated; }
    
    /// Return the pose of the frame in the (w)orld coordinate frame.
    inline Vector3d pos() const { return T_w_b.translation(); }    
    
    /// Return the rotation of the frame in the (w)orld coordinate frame.
//     inline Vector3d rot() const { return T_w_b.rotation_matrix(); }     
    

    
    /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
    inline Vector3d c2f(const Vector2d& px) const 
    { 
	Vector3d f;
	cam->liftProjective(px, f);
	return f.normalized(); 
    }

    inline Vector3d c2f(const double& x, const double& y) const 
    { 
	Vector2d px(x, y);
	return c2f(px); 
    }
    
    /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
    inline Vector2d w2c(const Vector3d& xyz_w) const 
    {
	Vector2d px;
	cam->spaceToPlane(w2f(xyz_w), px);
	return px;
    }
    
    /// Projects Point from unit sphere (f) in camera pixels (c).
    inline Vector2d f2c(const Vector3d& f) const 
    {
	Vector2d px;
	cam->spaceToPlane(f, px);// f << f(0) / f(2), f(1) / f(2);
	return px;	
    }
// charles    
    inline Vector2d f2c(const Vector2d& f) const
    {
	Vector3d f_lifted(f.x(), f.y(), 1.0);
	return f2c(f_lifted);
    }

    inline Vector3d c2b (const Vector3d& f) const
    {
	return RIC[0] * f + TIC[0];
    }
    
    inline Vector3d b2c (const Vector3d& xyz_b) const
    {
	return RIC[0].inverse() * (xyz_b - TIC[0]);
    }
    
    inline void c2b (Vector3d& P_w_c, Matrix3d& R_w_c) const
    {
	R_w_c = R_w_c * RIC[0].inverse();	
	P_w_c = P_w_c - R_w_c * TIC[0];
    }
    
    inline void b2c (Vector3d& P_w_b, Matrix3d& R_w_b) const
    {
	P_w_b = P_w_b + R_w_b * TIC[0];
	R_w_b = R_w_b * RIC[0];
    }

    /// Transforms point coordinates in world-frame (w) to camera-frams (f).
    inline Vector3d w2f(const Vector3d& xyz_w) const 
    {
	Vector3d xyz_b = T_w_b.inverse() * xyz_w;
	return b2c(xyz_b);
    }

    /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
    inline Vector3d f2w(const Vector3d& f) const 
    { 
	return T_w_b * c2b(f);
    }
    
    inline bool isInFrame(const Vector2i & obs, int boundary=0) const
    {
	if(obs[0]>=boundary && obs[0]<COL-boundary
	    && obs[1]>=boundary && obs[1]<ROW-boundary)
	    return true;
	return false;
    }    

    inline bool isInFrame(const Vector2i &obs, int boundary, int level) const
    {
	if(obs[0] >= boundary && obs[0] < COL/(1<<level)-boundary
	    && obs[1] >= boundary && obs[1] <ROW/(1<<level)-boundary)
	    return true;
	return false;
    }    
    
    inline static void jacobian_xyz2uv(
	const Vector3d& xyz_in_f,
	Matrix<double,2,6>& J)
    {
	const double x = xyz_in_f[0];
	const double y = xyz_in_f[1];
	const double z_inv = 1./xyz_in_f[2];
	const double z_inv_2 = z_inv*z_inv;

	J(0,0) = -z_inv;              // -1/z
	J(0,1) = 0.0;                 // 0
	J(0,2) = x*z_inv_2;           // x/z^2
	J(0,3) = y*J(0,2);            // x*y/z^2
	J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
	J(0,5) = y*z_inv;             // y/z

	J(1,0) = 0.0;                 // 0
	J(1,1) = -z_inv;              // -1/z
	J(1,2) = y*z_inv_2;           // y/z^2
	J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
	J(1,4) = -J(0,3);             // -x*y/z^2
	J(1,5) = -x*z_inv;            // x/z
    }		
    
    
    
    
};

}