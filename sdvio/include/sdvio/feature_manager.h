#pragma once

#include <global.h>
#include <point.h>

namespace sdvio 
{
using namespace Eigen;
    
class Feature;    
    
class FeatureManager : public Point
{
public:
    enum TriangulateType {
	TYPE_TRIANGULATED,
	TYPE_INITIAL
    };    
    
    TriangulateType	triang_type;
    double 		position[3];
    
    FeatureManager();
    FeatureManager(Feature* _ftr);
    virtual ~FeatureManager();
   
    void setPos(Vector3d _pos);
    void update();
};
    
}

