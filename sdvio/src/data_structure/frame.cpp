#include <frame.h>
#include <feature.h>
#include <point.h>
#include <fast/fast.h>

namespace sdvio 
{
int Frame::frame_counter = 0;

Frame::Frame(camodocal::CameraPtr cam, const cv::Mat& _img, double timestamp) :
    id(frame_counter++),
    window_counter(-1),
    timestamp(timestamp),
    cam(cam),
    is_keyframe(false),
    is_preintegrated(false)
{
    initFrame(_img);
}

Frame::Frame(camodocal::CameraPtr cam, const cv::Mat& _img, const vector< ImuMeasurement >& _imus, const Vector3d& _acc0, const Vector3d& _gyr0, const Vector3d& _ba0, const Vector3d& _bg0):
    id(frame_counter++),
    window_counter(-1),
    timestamp(0.0),
    cam(cam),
    is_keyframe(false),
    is_preintegrated(false)
{
    initFrame(_img);
    initIMU(_imus, _acc0, _gyr0, _ba0, _bg0);
}

Frame::Frame(camodocal::CameraPtr cam0, const cv::Mat& _img0, camodocal::CameraPtr cam1, const cv::Mat& _img1, const vector< ImuMeasurement >& _imus, const Vector3d& _acc0, const Vector3d& _gyr0, const Vector3d& _ba0, const Vector3d& _bg0):
    id(frame_counter++),
    window_counter(-1),
    timestamp(0.0),
    cam(cam0),
    cam1(cam1),
    is_keyframe(false),
    is_preintegrated(false)
{
    initFrameStereo(_img0, _img1);
    initIMU(_imus, _acc0, _gyr0, _ba0, _bg0);
}

Frame::~Frame()
{
    std::for_each(fts.begin(), fts.end(), [&](Feature* i){delete i;});
    delete preintegration;
} 
   
    
void Frame::addFeature(Feature* ftr)
{
    fts.push_back(ftr);
}

bool Frame::deleteFeature(Point* point)
{
    for (auto it = fts.begin(); it != fts.end(); ++it) {
	if ((*it)->point == point) {
	    fts.erase(it);
	    delete (*it);
	    return true;
	}
    }
    return false;
}


void Frame::setKeyframe()
{
    is_keyframe = true;
}


void Frame::initFrame(const cv::Mat& _img)
{
    // check image
    if(_img.empty() || _img.type() != CV_8UC1 || _img.cols != COL || _img.rows != ROW)
	throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

    // Build Image Pyramid
    createImgPyramid(_img, max(NUM_PYR_LEVELS, KLT_MAX_LEVEL+1), img_pyr);
}

void Frame::initFrameStereo(const cv::Mat& _img0, const cv::Mat& _img1)
{
    // check image
    if(_img0.empty() || _img0.type() != CV_8UC1 || _img0.cols != COL || _img0.rows != ROW)
	throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

    // Build Image Pyramid
    createImgPyramid(_img0, max(NUM_PYR_LEVELS, KLT_MAX_LEVEL+1), img_pyr);
    createImgPyramid(_img1, max(NUM_PYR_LEVELS, KLT_MAX_LEVEL+1), img_pyr1);
}

void Frame::initIMU(const vector< ImuMeasurement >& _imus, const Vector3d& _acc0, const Vector3d& _gyr0, const Eigen::Vector3d& _ba0, const Eigen::Vector3d& _bg0)
{
    preintegration = new IntegrationBase(_acc0, _gyr0, _ba0, _bg0);
    
    for (auto it = _imus.begin(); it != _imus.end(); ++it) {
	double pre_t = it->t.toSec();
	double next_t = (it+1)->t.toSec();
	double dt = next_t - pre_t;
	
	preintegration->push_back(dt, it->linear_acceleration, it->angular_velocity);
    }
    
    is_preintegrated = true;
}

void Frame::createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
    pyr.resize(n_levels);
    pyr[0] = img_level_0;
    for(int i=1; i<n_levels; ++i)
    {
	pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);
	utility::halfSample(pyr[i-1], pyr[i]);
    }
}

bool Frame::getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
    vector<double> depth_vec;
    depth_vec.reserve(frame.fts.size());
    depth_min = std::numeric_limits<double>::max();
    for(auto it=frame.fts.begin(), ite=frame.fts.end(); it!=ite; ++it)
    {
	if((*it)->point != NULL)
	{
	    const double z = frame.w2f((*it)->point->pos).z();
	    depth_vec.push_back(z);
	    depth_min = fmin(z, depth_min);
	}
    }
    if(depth_vec.empty())
    {
	//SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
	return false;
    }
    depth_mean = utility::getMedian(depth_vec);
    return true;
} 
    
void Frame::clear()
{
    if (fts.size() > 0) {
	std::for_each(fts.begin(), fts.end(), [](Feature* ftr){
	    ftr->point->deleteFrameRef(ftr->frame);
	    delete ftr;
	});
	fts.clear();	
    }
    delete preintegration;
}

}