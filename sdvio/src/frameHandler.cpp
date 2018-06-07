#include <frameHandler.h>
#include <feature_detector.h>
#include <feature.h>
#include <frame.h>
#include <point.h>
#include <utility.h>

namespace sdvio 
{
FrameHandler::FrameHandler():
    window_counter(-1),
    is_initialized(false)
{

}

FrameHandler::~FrameHandler()
{

}

// typedef list<Feature*> Features;
// void trackFeatures(
//     Frame* _ref_frame,
//     Frame* _cur_frame,
//     Features& _ref_features,
//     Features& _cur_features)
// {
//     _cur_features.clear();
//   
//     vector<cv::Point2f> ref_px;
//     vector<cv::Point2f> cur_px;
//     
//     std::for_each(_ref_features.begin(), _ref_features.end(), [&](Feature* ftr){
// 	ref_px.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
// // 	cur_px.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
//     });
// /*    
//     const double klt_win_size = 30.0;
//     const int klt_max_iter = 30;
//     const double klt_eps = 0.001;
//     vector<uchar> status;
//     vector<float> error;
//     vector<float> min_eig_vec;
//     cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
//     cv::calcOpticalFlowPyrLK(_ref_frame->img_pyr[0], _cur_frame->img_pyr[0],
// 			    ref_px, cur_px,
// 			    status, error,
// 			    cv::Size2i(klt_win_size, klt_win_size),
// 			    4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);  */
// 
//     vector<uchar> status;
//     vector<float> err;
//     cv::calcOpticalFlowPyrLK(_ref_frame->img_pyr[0],
// 			     _cur_frame->img_pyr[0], 
// 			     ref_px, cur_px, status, err, cv::Size(21, 21), 3);
//     
//     Features::iterator fts_ref_it = _ref_features.begin();
//     vector<cv::Point2f>::iterator ref_px_it = ref_px.begin();
//     vector<cv::Point2f>::iterator cur_px_it = cur_px.begin();
//     
//     for (size_t i = 0; ref_px_it != ref_px.end(); ++i) {
// 	if (!(status[i] && _cur_frame->isInFrame(Vector2i(ceil(cur_px_it->x), ceil(cur_px_it->y)), 1))) {
// // 	    delete (*fts_ref_it);
// 	    fts_ref_it = _ref_features.erase(fts_ref_it);
// 	    ref_px_it = ref_px.erase(ref_px_it);
// 	    cur_px_it = cur_px.erase(cur_px_it);
// 	    continue;
// 	} 
// 	_cur_features.push_back(new Feature(_cur_frame, Vector2d(cur_px_it->x, cur_px_it->y), (*fts_ref_it)->level));
// 	++fts_ref_it;
// 	++ref_px_it;
// 	++cur_px_it;
// 	
// // 	_cur_frame->addFeature(_cur_features.back());
//     }
// }

// void trackFeatures(
//     Frame* _ref_frame,
//     Frame* _cur_frame,
//     vector<cv::Point2f>& px_ref,
//     vector<cv::Point2f>& px_cur,
//     vector<double>& disparities,
//     vector<bool> & status
//   		)
// {
//     //TODO 特征点重新构建,一个一个用_ref_frame里的特征点来构建.vector<cv::Point2f>,这样如果跟踪上了,直接在相应特征点的位置加上新观测值
//     // Features果然用vector比较好.这样可以按照元素顺序操作.不过list也可以,遍历就好,一个一个看state是否为true,如果为true, 在该特征点的管理器上加上这一新观测,建立新特征点放到当前帧中
//     
//     
//     const double klt_win_size = 30.0;
//     const int klt_max_iter = 30;
//     const double klt_eps = 0.001;
//     vector<uchar> status;
//     vector<float> error;
//     vector<float> min_eig_vec;
//     cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
//     cv::calcOpticalFlowPyrLK(_ref_frame->img_pyr[0], _cur_frame->img_pyr[0],
// 			    px_ref, px_cur,
// 			    status, error,
// 			    cv::Size2i(klt_win_size, klt_win_size),
// 			    4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);  
// /*
//     vector<uchar> status;
//     vector<float> err;
//     cv::calcOpticalFlowPyrLK(_ref_frame->img_pyr[0],
// 			     _cur_frame->img_pyr[0], 
// 			     px_ref, px_cur, status, err, cv::Size(21, 21), 3);*/
//     
//     vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
//     vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
//     
//     for (size_t i = 0; px_ref_it != px_ref.end(); ++i) {
// 	if (!(status[i] && _cur_frame->isInFrame(Vector2i(ceil(px_cur_it->x), ceil(px_cur_it->y)), 1))) {
// 	    px_ref_it = px_ref.erase(px_ref_it);
// 	    px_cur_it = px_cur.erase(px_cur_it);
// 	    continue;
// 	} 
// 	disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
// 	++px_ref_it;
// 	++px_cur_it;
//     }
// }

void trackFeatures(
    Frame* _ref_frame,
    Frame* _cur_frame,
    vector<double>& disparities)
{
    
    //TODO 特征点重新构建,一个一个用_ref_frame里的特征点来构建.vector<cv::Point2f>,这样如果跟踪上了,直接在相应特征点的位置加上新观测值
    // Features果然用vector比较好.这样可以按照元素顺序操作.不过list也可以,遍历就好,一个一个看state是否为true,如果为true, 在该特征点的管理器上加上这一新观测,建立新特征点放到当前帧中
    
    vector<cv::Point2f> px_ref;
    vector<cv::Point2f> px_cur;
    
    std::for_each(_ref_frame->fts.begin(), _ref_frame->fts.end(), [&](Feature* ftr){
	px_ref.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
// 	px_cur.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    });
    
    const double klt_win_size = 30.0;
    const int klt_max_iter = 30;
    const double klt_eps = 0.001;
    vector<uchar> status;
    vector<float> error;
    vector<float> min_eig_vec;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
//     cv::calcOpticalFlowPyrLK(_ref_frame->img_pyr[0], _cur_frame->img_pyr[0],
// 			    px_ref, px_cur,
// 			    status, error,
// 			    cv::Size2i(klt_win_size, klt_win_size),
// 			    4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);  

cv::calcOpticalFlowPyrLK(_ref_frame->img_pyr[0], _cur_frame->img_pyr[0], 
			 px_ref, px_cur, status, error, cv::Size(21, 21), 3);
    
    vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
    vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
    Features::iterator fts_ref_it = _ref_frame->fts.begin();
    for (size_t i = 0; px_ref_it != px_ref.end(); ++i, ++px_ref_it, ++px_cur_it, ++fts_ref_it) {
	if (!(status[i] && _cur_frame->isInFrame(Vector2i(ceil(px_cur_it->x), ceil(px_cur_it->y)), 1))) 
	    continue;
	
	_cur_frame->addFeature(new Feature(_cur_frame, Vector2d(px_cur_it->x, px_cur_it->y), (*fts_ref_it)->level));
	(*fts_ref_it)->point->addFrameRef(_cur_frame->fts.back());
	_cur_frame->fts.back()->point = (*fts_ref_it)->point;
			
	disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    }  
}


bool test_flag = false;
void FrameHandler::addFrame(Frame* _new_frame)
{
    if (!isInitialized()) {
	if (window_counter < WINDOW_SIZE-1)
	    window_counter++;
	frames[window_counter].reset(_new_frame);
	
	//Initiate detector
	feature_detection::FastDetector detector(
	    frames[window_counter]->img().cols,
	    frames[window_counter]->img().rows,
	    100, 
	    1);    
	
	// Track old features
	if (window_counter > 0 && !frames[window_counter-1]->fts.empty()) {
	    ROS_INFO_STREAM("FrameHandler------addFrame: fts_ref size: " << frames[window_counter-1]->fts.size() );
	    ROS_INFO_STREAM("FrameHandler------addFrame: fts_cur size: " << frames[window_counter]->fts.size() );
	    vector<double> disparities;
	    trackFeatures(
		frames[window_counter-1].get(), 
		frames[window_counter].get(),
		disparities); 
	    detector.setExistingFeatures(frames[window_counter]->fts);
// 	    if (disparities.size() < 20) {
// 		
// 	    }
// 	    double disparity = utility::getMedian(disparities);
	    
	    ROS_INFO_STREAM("FrameHandler------addFrame: tracked_features size: " << frames[window_counter]->fts.size() );

	}   
	
	if(!test_flag) {
	    test_flag = true;
	    Features feature_new;
	    
	    detector.detect(frames[window_counter].get(), frames[window_counter]->img_pyr, TRIANG_MIN_CORNER_SCORE , feature_new); 
	    
	    // Create new FeatureManager
	    std::for_each(feature_new.begin(), feature_new.end(), [&](Feature* ftr){
		frames[window_counter]->addFeature(new Feature(frames[window_counter].get(), ftr->px, ftr->level));
		feature_managers.push_back(new FeatureManager(frames[window_counter]->fts.back()));
		frames[window_counter]->fts.back()->point = feature_managers.back();
		delete ftr;
	    });	    
	    
	    ROS_INFO_STREAM("FrameHandler------addFrame: new_features size: " << frames[window_counter]->fts.size() );
	}		
	
    } else {
// 	// Return relative R,t between frame_last and frame_new, set keyframe flag
// 	trackFrame(frame_last, _new_frame);
// 	
// 	// Align Features
// 	
// 	// Update depth filter
// 	if (_new_frame->isKeyframe()) 
// 	    depth_filter->addKeyframe();
// 	else
// 	    depth_filter->addFrame();
// 
// 	frames[window_counter].reset(_new_frame);
// 	frame_last = _new_frame;
    }

    cv::Mat img = frames[window_counter]->img_pyr[0];
    
    for (auto ftr : frames[window_counter]->fts) {
	cv::Point2f px(ftr->px.x(), ftr->px.y());
	cv::circle(img, px, 2, cv::Scalar(0xff, 0xff, 0xff), 2);
    }

    cv::imshow("view", img);
    cv::waitKey(5);    
}


	
/*    
fts_ref.clear();
std::for_each(fts_cur.begin(), fts_cur.end(), [&](Feature* ftr){
    fts_ref.push_back(ftr);
    delete ftr;
});
fts_cur.clear();*/

/*	
std::for_each(fts_ref.begin(), fts_ref.end(), [&](Feature* ftr){
    delete ftr;
});
fts_ref.clear();

std::for_each(fts_cur.begin(), fts_cur.end(), [&](Feature* ftr){
    fts_ref.push_back(new Feature(ftr->frame, ftr->px, ftr->level));
    delete ftr;
});
fts_cur.clear();*/

/*
 * 
 * vector<cv::Point2f> ref_px;
vector<cv::Point2f> cur_px;

    // Track old features
    if (!ref_px.empty()) {
	cur_px.clear();
	vector<uchar> status;
	vector<float> err;
	cv::calcOpticalFlowPyrLK(frames[window_counter-1]->img(), frames[window_counter]->img(), ref_px, cur_px, status, err, cv::Size(21, 21), 3);

	vector<cv::Point2f>::iterator ref_px_it = ref_px.begin();
	vector<cv::Point2f>::iterator cur_px_it = cur_px.begin();
	
	for (size_t i = 0; ref_px_it != ref_px.end(); ++i, ++ref_px_it, ++cur_px_it) {
	    if (!status[i] || !frames[window_counter]->isInFrame(Vector2i(ceil(cur_px_it->x), ceil(cur_px_it->y)), 1)) {
		ref_px_it = ref_px.erase(ref_px_it);
		cur_px_it = cur_px.erase(cur_px_it);
		continue;
	    } 
	}
// 	detector.setExistingFeatures(fts_cur);
    }     
    
    if(!test_flag) {
	test_flag = true;
// 	cv::Mat img_gray;
// 	cv::cvtColor(frames[window_counter]->img(), img_gray, CV_BGR2GRAY);
	cv::goodFeaturesToTrack(frames[window_counter]->img(), cur_px, 100, 0.01, GRID_SIZE);    
    }
    
    cv::Mat img = frames[window_counter]->img();
    for (auto px : cur_px) {
	cv::circle(img, px, 2, cv::Scalar(0xff, 0xff, 0xff), 2);
    }    
    
    ref_px = cur_px;

    */

bool FrameHandler::trackFrame(Frame* _new_frame)
{
    frame_new.reset(_new_frame);
    
    frame_new->T_w_b = frame_last->T_w_b;// multiplied by IMU preintegration
    
//     SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
//                            30, SparseImgAlign::GaussNewton, false, false);
//     
//     size_t img_align_n_tracked = img_align.run(last_frame, new_frame);
    
    // 如果跟踪成功,判断是否有共同的观测点,如果找到足够多的match point,则视为成功
    
    // 用图优化对特征点们再进行一个优化
    return true;
    // 
 
}

void FrameHandler::slideOldFrame()
{
    for (int i = 0; i < WINDOW_SIZE-1; ++i) {
	frames[i].swap(frames[i+1]);
	frames[i]->setWindowCounter(i);
    }
//     frames[WINDOW_SIZE-1]->clear();
}

}