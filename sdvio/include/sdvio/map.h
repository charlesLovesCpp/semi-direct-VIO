#pragma once

#include <global.h>
#include <queue>
#include <set>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>


namespace sdvio 
{
    
class Point;
class Feature;
class Seed;

class MapPointCandidates
{
public:
    typedef pair<Point*, Feature*> PointCandidate;
    typedef list<PointCandidate> PointCandidateList;
    
    boost::mutex mut;

    PointCandidateList candidates;
    list<Point*> trash_points;
    
    MapPointCandidates();
    ~MapPointCandidates();    
    
    /// Reset the candidate list, remove and delete all points.
    void reset();

    void deleteCandidate(PointCandidate& c);

    void emptyTrash();
};

class Map : boost::noncopyable
{
public:
    list<FramePtr> keyframes;          //!< List of keyframes in the map.
    list<Point*> trash_points;         //!< A deleted point is moved to the trash bin. Now and then this is cleaned. One reason is that the visualizer must remove the points also.
    MapPointCandidates point_candidates;

    Map();
    ~Map();    
    
    /// Reset the map. Delete all keyframes and reset the frame and point counters.
    void reset();   
  
    void emptyTrash();
};

}