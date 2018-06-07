#include <point.h>
#include <feature.h>
#include <frame.h>

namespace sdvio 
{
    
int Point::point_counter = 0;   

Point::Point(const Vector3d& pos) :
    id(point_counter++),
    pos(pos),
    normal_set(false),
    n_obs(0),
    type(TYPE_UNKNOWN),
    n_failed_reproj(0),	
    n_succeeded_reproj(0)
{}

Point::Point(const Vector3d& pos, Feature* ftr) :
    id(point_counter++),
    pos(pos),
    normal_set(false),
    n_obs(1),
    type(TYPE_UNKNOWN),
    n_failed_reproj(0),
    n_succeeded_reproj(0)
{
    obs.push_front(ftr);
}

Point::~Point()
{}

void Point::addFrameRef(Feature* ftr)
{
    obs.push_front(ftr);
    ++n_obs;
}

Feature* Point::findFrameRef(Frame* frame)
{
    for(auto it=obs.begin(), ite=obs.end(); it!=ite; ++it)
	if((*it)->frame == frame)
	return *it;
    return NULL;    // no keyframe found
}

bool Point::deleteFrameRef(Frame* frame)
{
    for(auto it=obs.begin(), ite=obs.end(); it!=ite; ++it) {
	if((*it)->frame == frame) {
	obs.erase(it);
	return true;
	}
    }
    return false;
}

bool Point::getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const
{
    // TODO: get frame with same point of view AND same pyramid level!
    Vector3d obs_dir(framepos - pos); obs_dir.normalize();
    auto min_it=obs.begin();
    double min_cos_angle = 0;
    for(auto it=obs.begin(), ite=obs.end(); it!=ite; ++it) {
	Vector3d dir((*it)->frame->pos() - pos); dir.normalize();
	double cos_angle = obs_dir.dot(dir);
	if(cos_angle > min_cos_angle) {
	min_cos_angle = cos_angle;
	min_it = it;
	}
    }
    ftr = *min_it;
    if(min_cos_angle < 0.5) // assume that observations larger than 60° are useless
	return false;
    return true;
}

}