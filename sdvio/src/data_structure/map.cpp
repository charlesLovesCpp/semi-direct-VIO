#include <map.h>
#include <point.h>
#include <frame.h>
#include <feature.h>
#include <boost/bind.hpp>

namespace sdvio 
{
    
MapPointCandidates::MapPointCandidates()
{}

MapPointCandidates::~MapPointCandidates()
{
    reset();
}

void MapPointCandidates::reset()
{
    boost::unique_lock<boost::mutex> lock(mut);
    std::for_each(candidates.begin(), candidates.end(), [&](PointCandidate& c){
    delete c.first;
    delete c.second;
    });
    candidates.clear();
}

void MapPointCandidates::deleteCandidate(PointCandidate& c)
{
    // camera-rig: another frame might still be pointing to the candidate point
    // therefore, we can't delete it right now.
    delete c.second; c.second=NULL;
    c.first->type = Point::TYPE_DELETED;
    trash_points.push_back(c.first);
}

void MapPointCandidates::emptyTrash()
{
    std::for_each(trash_points.begin(), trash_points.end(), [&](Point*& p){
    delete p; p=NULL;
    });
    trash_points.clear();
}

Map::Map() 
{}

Map::~Map()
{
    reset();
}    

void Map::reset()
{
    keyframes.clear();
    point_candidates.reset();
    emptyTrash();
}

void Map::emptyTrash()
{
  std::for_each(trash_points.begin(), trash_points.end(), [&](Point*& pt){
    delete pt;
    pt=NULL;
  });
  trash_points.clear();
  point_candidates.emptyTrash();
}

}