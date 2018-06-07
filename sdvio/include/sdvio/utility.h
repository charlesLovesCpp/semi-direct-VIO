#pragma once

#include <cmath>
#include <cassert>
#include <cstring>
#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <map>
#include <string>
#include <iostream>
#include <fstream>

#include <global.h>


using namespace Eigen;

namespace sdvio {
    
class Frame;  
class Feature;
typedef list<Feature*> Features;

namespace utility {
 
Vector3d triangulateFeatureNonLin(
	const Matrix3d& R,
	const Vector3d& t,
	const Vector3d& feature1,
	const Vector3d& feature2);

double reprojError(
	const Vector3d& f1,
	const Vector3d& f2,
	double focal_length);

double computeInliers(
	const std::vector<Vector3d>& features1,
	const std::vector<Vector3d>& features2,
	const Matrix3d& R,
	const Vector3d& t,
	const double reproj_thresh,
	double focal_length,
	std::vector<Vector3d>& xyz_vec,
	std::vector<int>& inliers,
	std::vector<int>& outliers);

double sampsonusError(
	const Vector2d &v2Dash,
	const Matrix3d& m3Essential,
	const Vector2d& v2);

void getCorrespondingPoints(Frame* frame_i, Frame* frame_j, vector< pair< Feature*, Feature* > >& _matches);

void computeDistanceByImg(const Vector3d& pt_1, const Vector3d& pt_2, double& parallex);

bool computeParallex(Frame* frame_i, Frame* frame_j, double& _parallex);

void triangulatePoint(
    const Eigen::Matrix<double, 3, 4>& Pose_i, 
    const Eigen::Matrix<double, 3, 4>& Pose_j,
    const Vector3d& point_i, 
    const Vector3d& point_j, 
    Vector3d& pos);

void trackFeatures(
    Frame* _ref_frame,
    Frame* _cur_frame,
    Features& _ref_features,
    Features& _cur_features);

void halfSample(const cv::Mat& in, cv::Mat& out);

template<class T>
inline T getMedian(std::vector<T>& data_vec)
{
	assert(!data_vec.empty());
	typename std::vector<T>::iterator it = data_vec.begin() + floor(data_vec.size() / 2);
	nth_element(data_vec.begin(), it, data_vec.end());//对前n个数进行排序
	return *it;
}

#ifdef __SSE2__
inline void halfSampleSSE2(const unsigned char* in, unsigned char* out, int w, int h)
{
  const unsigned long long mask[2] = { 0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull };
  const unsigned char* nextRow = in + w;
  __m128i m = _mm_loadu_si128((const __m128i*)mask);
  int sw = w >> 4;
  int sh = h >> 1;
  for (int i = 0; i < sh; i++)
  {
    for (int j = 0; j < sw; j++)
    {
      __m128i here = _mm_load_si128((const __m128i*)in);
      __m128i next = _mm_load_si128((const __m128i*)nextRow);
      here = _mm_avg_epu8(here, next);
      next = _mm_and_si128(_mm_srli_si128(here, 1), m);
      here = _mm_and_si128(here, m);
      here = _mm_avg_epu16(here, next);
      _mm_storel_epi64((__m128i*)out, _mm_packus_epi16(here, here));
      in += 16;
      nextRow += 16;
      out += 8;
    }
    in += w;
    nextRow += w;
  }
}
#endif 

inline Vector3d p2f(const Vector3d& f)
{
    return f.normalized();
}

/// 投影，摄像机坐标系下坐标转像素世界坐标
inline Vector2d project2d(const Vector3d& v)
{
	return v.head<2>() / v[2];
}

/// 反投影，将像素的世界坐标转相对于的摄像机坐标
inline Vector3d unproject2d(const Vector2d& v)
{
	return Vector3d(v[0], v[1], 1.0);
}
///给出反对称矩阵
inline Matrix3d sqew(const Vector3d& v)
{
	Matrix3d v_sqew;
	v_sqew << 0, -v[2], v[1],
		v[2], 0, -v[0],
		-v[1], v[0], 0;
	return v_sqew;
}

inline double norm_max(const Eigen::VectorXd & v)
{
	double max = -1;
	for (int i = 0; i < v.size(); i++)
	{
		double abs = fabs(v[i]);
		if (abs > max){
			max = abs;
		}
	}
	return max;
}

//     Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);
inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}


Eigen::Matrix3d g2R(const Eigen::Vector3d &g);
    
//! Return value between 0 and 1
//! WARNING This function does not check whether the x/y is within the border
inline float
interpolateMat_32f(const cv::Mat& mat, float u, float v)
{
    assert(mat.type()==CV_32F);
    float x = floor(u);
    float y = floor(v);
    float subpix_x = u-x;
    float subpix_y = v-y;
    float wx0 = 1.0-subpix_x;
    float wx1 =  subpix_x;
    float wy0 = 1.0-subpix_y;
    float wy1 =  subpix_y;

    float val00 = mat.at<float>(y,x);
    float val10 = mat.at<float>(y,x+1);
    float val01 = mat.at<float>(y+1,x);
    float val11 = mat.at<float>(y+1,x+1);
    return (wx0*wy0)*val00 + (wx1*wy0)*val10 + (wx0*wy1)*val01 + (wx1*wy1)*val11;
}

//! Return value between 0 and 255
//! WARNING This function does not check whether the x/y is within the border
inline float
interpolateMat_8u(const cv::Mat& mat, float u, float v)
{
    assert(mat.type()==CV_8U);
    int x = floor(u);
    int y = floor(v);
    float subpix_x = u-x;
    float subpix_y = v-y;

    float w00 = (1.0f-subpix_x)*(1.0f-subpix_y);
    float w01 = (1.0f-subpix_x)*subpix_y;
    float w10 = subpix_x*(1.0f-subpix_y);
    float w11 = 1.0f - w00 - w01 - w10;

    const int stride = mat.step.p[0];
    unsigned char* ptr = mat.data + y*stride + x;
    return w00*ptr[0] + w01*ptr[stride] + w10*ptr[1] + w11*ptr[stride+1];
}
    
void calcSharrDeriv(const cv::Mat& src, cv::Mat& dst);

// turn a 3D angle vector into a quaternion.
template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
	q(2), typename Derived::Scalar(0), -q(0),
	-q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
{
    //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
    //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
    //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
{
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
    return ans;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
{
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
    return ans;
}
    
    template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }



    template <size_t N>
    struct uint_
    {
    };

    template <size_t N, typename Lambda, typename IterT>
    inline void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    template <typename Lambda, typename IterT>
    inline void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }

    template <typename T>
    inline T normalizeAngle(const T& angle_degrees) {
      T two_pi(2.0 * 180);
      if (angle_degrees > 0)
      return angle_degrees -
          two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
        return angle_degrees +
            two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    };
    

}// end utility

}// end sdvio
