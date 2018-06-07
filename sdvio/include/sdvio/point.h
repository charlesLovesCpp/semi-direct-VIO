#pragma once

#include <global.h>
#include <boost/noncopyable.hpp>

namespace sdvio 
{
    
typedef Matrix<double, 2, 3> Matrix23d;

class Feature;
class Frame;  

class Point : boost::noncopyable
{
public:
    enum PointType {
    TYPE_DELETED,
    TYPE_CANDIDATE,
    TYPE_UNKNOWN,
    TYPE_GOOD
    };
  
    static int			point_counter;
    int				id;
    Vector3d                    pos;                     //!< 3d pos of the point in the world coordinate frame.
    Vector3d                    normal;                  //!< Surface normal at point.
    Matrix3d                    normal_information;      //!< Inverse covariance matrix of normal estimation.
    bool                        normal_set;              //!< Flag whether the surface normal was estimated or not.
    list<Feature*>              obs;                     //!< References to keyframes which observe the point.
    size_t                      n_obs;                   //!< Number of obervations: Keyframes AND successful reprojections in intermediate frames.
    PointType                   type;                    //!< Quality of the point.
    int                         n_failed_reproj;         //!< Number of failed reprojections. Used to assess the quality of the point.
    int                         n_succeeded_reproj;      //!< Number of succeeded reprojections. Used to assess the quality of the point.
    
    Point(const Vector3d& _pos);
    Point(const Vector3d& _pos, Feature* _ftr);
    virtual ~Point();
    
    Vector3d getPos() { return pos; };
    
    /// Add a reference to a frame.
    void addFrameRef(Feature* ftr);

    /// Remove reference to a frame.
    bool deleteFrameRef(Frame* frame);    
    
    /// Check whether mappoint has reference to a frame.
    Feature* findFrameRef(Frame* frame);
    
    /// Get Frame with similar viewpoint.
    bool getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const;
    
    /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
    inline static void jacobian_xyz2uv(
	const Vector3d& _p_in_f,
	const Matrix3d& _R_f_w,
	Matrix23d& point_jac)
    {
	const double z_inv = 1.0/_p_in_f[2];
	const double z_inv_sq = z_inv*z_inv;
	point_jac(0, 0) = z_inv;
	point_jac(0, 1) = 0.0;
	point_jac(0, 2) = -_p_in_f[0] * z_inv_sq;
	point_jac(1, 0) = 0.0;
	point_jac(1, 1) = z_inv;
	point_jac(1, 2) = -_p_in_f[1] * z_inv_sq;
	point_jac = - point_jac * _R_f_w;
    }
};



}