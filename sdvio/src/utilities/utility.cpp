#include <utility.h>
#include <feature.h>
#include <frame.h>
#include <point.h>

namespace sdvio
{
namespace utility 
{
Vector3d triangulateFeatureNonLin(const Matrix3d& R, const Vector3d& t,//输入是转换矩阵R，t，还有两个对应的3D特征点坐标，应该是相机坐标系上的
      const Vector3d& feature1, const Vector3d& feature2)
{
    Vector3d f2 = R * feature2;
    Vector2d b;
    b[0] = t.dot(feature1);
    b[1] = t.dot(f2);
    Matrix2d A;
    A(0, 0) = feature1.dot(feature1);
    A(1, 0) = feature1.dot(f2);
    A(0, 1) = -A(1, 0);
    A(1, 1) = -f2.dot(f2);
    Vector2d lambda = A.inverse() * b;
    Vector3d xm = lambda[0] * feature1;
    Vector3d xn = t + lambda[1] * f2;
    return (xm + xn) / 2;
}

double reprojError(const Vector3d& f1,
      const Vector3d& f2,
      double focal_length)
{
    Vector2d e = project2d(f1) - project2d(f2);//光流法计算的3D点和三角法计算的3D点的重投影误差
    return focal_length * e.norm();
}

double computeInliers(const std::vector<Vector3d>& features1, // c1
      const std::vector<Vector3d>& features2, // c2
      const Matrix3d& R,                 // R_c1_c2
      const Vector3d& t,                 // c1_t
      const double reproj_thresh,
      double focal_length,
      std::vector<Vector3d>& xyz_vec,         // in frame c1
      std::vector<int>& inliers,
      std::vector<int>& outliers)
{
    inliers.clear(); inliers.reserve(features1.size());
    outliers.clear(); outliers.reserve(features1.size());
    xyz_vec.clear(); xyz_vec.reserve(features1.size());
    double tot_error = 0;
    //三角化所有特征，然后计算投影误差和内点
    for (size_t j = 0; j<features1.size(); ++j)
    {
	xyz_vec.push_back(triangulateFeatureNonLin(R, t, features1[j], features2[j]));//利用两个特征点，求得深度值，得到3D点
	double e1 = reprojError(features1[j], xyz_vec.back(), focal_length);
	double e2 = reprojError(features2[j], R.transpose()*(xyz_vec.back() - t), focal_length);//equivalevent to T^-1 * xyz_vec. Transform xyz_vec back to c2 reference frame.
	if (e1 > reproj_thresh || e2 > reproj_thresh)//如果在两个图像中的重投影误差均大于门限，则视为outliers
	    outliers.push_back(j);
	else
	{
	    inliers.push_back(j);
	    tot_error += e1 + e2;
	}
    }
    return tot_error;
}

double sampsonusError(const Vector2d &v2Dash, const Matrix3d& essential, const Vector2d& v2)
{
    Vector3d v3Dash = unproject2d(v2Dash);
    Vector3d v3 = unproject2d(v2);

    double error = v3Dash.transpose() * essential * v3;//essential matrix error, which should be 0 when the point perfectly aligns.

    Vector3d fv3 = essential * v3;
    Vector3d fTv3Dash = essential.transpose() * v3Dash;

    Vector2d fv3Slice = fv3.head<2>();
    Vector2d fTv3DashSlice = fTv3Dash.head<2>();

    return (error * error / (fv3Slice.dot(fv3Slice) + fTv3DashSlice.dot(fTv3DashSlice))); // The definition of sampsonus error 
}

void halfSample(const cv::Mat& in, cv::Mat& out)
{
    assert( in.rows/2==out.rows && in.cols/2==out.cols);
    assert( in.type()==CV_8U && out.type()==CV_8U);
/*    
#ifdef __SSE2__
  if (is_aligned16(in.data) && is_aligned16(out.data) && ((in.cols % 16) == 0))
  {
    halfSampleSSE2(in.data, out.data, in.cols, in.rows);
    return;
  }
#endif */

    const int stride = in.step.p[0];
    uint8_t* top = (uint8_t*) in.data;
    uint8_t* bottom = top + stride;
    uint8_t* end = top + stride*in.rows;
    const int out_width = out.cols;
    uint8_t* p = (uint8_t*) out.data;
    while (bottom < end)
    {
	for (int j=0; j<out_width; j++)
	{
	    *p = static_cast<uint8_t>( (uint16_t (top[0]) + top[1] + bottom[0] + bottom[1])/4 );
	    p++;
	    top += 2;
	    bottom += 2;
	}
	top += stride;
	bottom += stride;
    }
}    

Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = R2ypr(R0).x();
    R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    return R0;
}

void getCorrespondingPoints(Frame* frame_i, Frame* frame_j, vector< pair< Feature*, Feature* > >& _matches)
{
    vector<pair<Feature*, Feature*>> matches;
    auto it = frame_i->fts.begin(); 
    auto it_end = frame_i->fts.end();
    for (;it != it_end; ++it) {
	Feature* ftr_matched = (*it)->point->findFrameRef(frame_j);
	if (!ftr_matched) {
	    continue;
	} else {
	    matches.push_back(std::make_pair(*it, ftr_matched));
	}
    }
    _matches = matches;    
}

void computeDistanceByImg(const Vector3d& pt_1, const Vector3d& pt_2, double& parallex)
{
    Vector2d dist = pt_1.head<2>() / pt_1.z() - pt_2.head<2>() / pt_2.z();
    parallex =  dist.norm();
}

bool computeParallex(Frame* frame_i, Frame* frame_j, double& _parallex)
{
    vector<pair<Feature*, Feature*>> matches;
    getCorrespondingPoints(frame_i, frame_j, matches);
    
    if (matches.size() > 20) {
	double sum_parallex = 0.0;
	for (auto& m_pt : matches) {
	    double parallex;
	    computeDistanceByImg(m_pt.first->f, m_pt.second->f, parallex);
	    sum_parallex += parallex;
	}
	_parallex = sum_parallex / matches.size();
	return true;
    } else {
	ROS_WARN("Initialization------computeParallex: no enough matched points");
	return false;	
    }
}

void triangulatePoint(
    const Eigen::Matrix<double, 3, 4>& Pose_i, 
    const Eigen::Matrix<double, 3, 4>& Pose_j,
    const Vector3d& point_i, 
    const Vector3d& point_j, 
    Vector3d& pos)
{
    Matrix4d design_matrix = Matrix4d::Zero();
    design_matrix.row(0) = point_i[0] * Pose_i.row(2) - point_i[2] * Pose_i.row(0);
    design_matrix.row(1) = point_i[1] * Pose_i.row(2) - point_i[2] * Pose_i.row(1);
    design_matrix.row(2) = point_j[0] * Pose_j.row(2) - point_j[2] * Pose_j.row(0);
    design_matrix.row(3) = point_j[1] * Pose_j.row(2) - point_j[2] * Pose_j.row(1);
    Vector4d triangulated_point;
    triangulated_point =
		    design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    pos(0) = triangulated_point(0) / triangulated_point(3);
    pos(1) = triangulated_point(1) / triangulated_point(3);
    pos(2) = triangulated_point(2) / triangulated_point(3);
}

typedef list<Feature*> Features;
void trackFeatures(
    Frame* _ref_frame,
    Frame* _cur_frame,
    Features& _ref_features,
    Features& _cur_features)
{
    _cur_features.clear();
  
    vector<cv::Point2f> ref_px;
    vector<cv::Point2f> cur_px;
    
    std::for_each(_ref_features.begin(), _ref_features.end(), [&](Feature* ftr){
	ref_px.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
	cur_px.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    });
    
    const double klt_win_size = 30.0;
    const int klt_max_iter = 30;
    const double klt_eps = 0.001;
    vector<uchar> status;
    vector<float> error;
    vector<float> min_eig_vec;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
    cv::calcOpticalFlowPyrLK(_ref_frame->img_pyr[0], _cur_frame->img_pyr[0],
			    ref_px, cur_px,
			    status, error,
			    cv::Size2i(klt_win_size, klt_win_size),
			    4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);  
    
    ROS_INFO_STREAM("utility------trackFeatures: calcOpticalFlowPyrLK()");
    
    Features::iterator ref_fts_it = _ref_features.begin();
    vector<cv::Point2f>::iterator ref_px_it = ref_px.begin();
    vector<cv::Point2f>::iterator cur_px_it = cur_px.begin();
    
    for (size_t i = 0; ref_px_it != ref_px.end(); ++i) {
	if (!status[i]) {
// 	    delete (*ref_fts_it);
	    ref_fts_it = _ref_features.erase(ref_fts_it);
	    ref_px_it = ref_px.erase(ref_px_it);
	    cur_px_it = cur_px.erase(cur_px_it);
	    continue;
	}
	ROS_INFO_STREAM("utility------trackFeatures: add tracked features to current features");	
	_cur_features.push_back(new Feature(_cur_frame, Vector2d(cur_px_it->x, cur_px_it->y), (*ref_fts_it)->level));
// 	_cur_frame->addFeature(_cur_features.back());
    }
}

}// end utility

}




