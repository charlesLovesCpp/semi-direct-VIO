#include <feature_manager.h>
#include <feature.h>
#include <frame.h>
#include <point.h>

namespace sdvio 
{
    
FeatureManager::FeatureManager(): 
    Point(Vector3d(-1,-1,-1)),
    triang_type(TYPE_INITIAL)
{
}


FeatureManager::FeatureManager(Feature* _ftr): 
    Point(Vector3d(-1,-1,-1)),
    triang_type(TYPE_INITIAL)
{
    addFrameRef(_ftr);
}

FeatureManager::~FeatureManager()
{}


void FeatureManager::setPos(Vector3d _pos)
{
    pos = _pos;
    position[0] = pos(0);
    position[1] = pos(1);
    position[2] = pos(2);
}

void FeatureManager::update()
{
    pos(0) = position[0];
    pos(1) = position[1];
    pos(2) = position[2];    
}

}