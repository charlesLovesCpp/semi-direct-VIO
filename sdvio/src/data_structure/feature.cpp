#include <feature.h>
#include <frame.h>
#include <point.h>

namespace sdvio 
{
int Feature::feature_counter = 0;    
    
Feature::Feature(Frame* _frame, const Vector2d& _px, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(frame->c2f(px)),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
{} 

Feature::Feature(Frame* _frame, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
{}

Feature::Feature(Frame* _frame, Point* _point, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(_point),
    grad(1.0,0.0)
{}     
/*
Feature::Feature(Frame* _frame, const Vector2d& _px, int _level, bool _haveID):
    type(CORNER),
    id(feature_counter++),
    frame(_frame),
    px(_px),
    f(frame->c2f(px)),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
{}



Feature::Feature(Frame* _frame, const Vector2d& _px, int _level, int _id):
    type(CORNER),
    frame(_frame),
    px(_px),
    f(frame->c2f(px)),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
{
    id = _id;
}*/

}