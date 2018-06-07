#pragma once
#include <global.h>

namespace sdvio
{
    
class Frame;
class Point;

class Feature
{
public:
    enum FeatureType {
    CORNER,
    EDGELET
    };    
    
    static int		feature_counter;
    FeatureType 	type;     //!< Type can be corner or edgelet.
    Frame* 		frame;         //!< Pointer to frame in which the feature was detected.
//     int 		id;
    Vector2d 		px;          //!< Coordinates in pixels on pyramid level 0.
    Vector3d 		f;           //!< Unit-bearing vector of the feature.
    int 		level;            //!< Image pyramid level where feature was extracted.
    Point*		point;         //!< Pointer to 3D point which corresponds to the feature.
    Vector2d 		grad;        //!< Dominant gradient direction for edglets, normalized.    
    
    Feature(Frame* _frame, const Vector2d& _px, int _level);
    
    Feature(Frame* _frame, const Vector2d& _px, const Vector3d& _f, int _level);
    
    Feature(Frame* _frame, Point* _point, const Vector2d& _px, const Vector3d& _f, int _level);
    
    Vector2d getPtc() { 
	return (f / f.z()).head<2>(); 
    };
    
//     Feature(Frame* _frame, const Vector2d& _px, int _level, bool _haveID);
//     
//     Feature(Frame* _frame, const Vector2d& _px, int _level, int _id);
};
    
}