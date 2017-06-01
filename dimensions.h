#ifndef DIMENSIONS_H
#define DIMENSIONS_H

// Basically hardcode everything here, unit is meter
template <unsigned int SCALE>
class robot_dimensions
{
public:
    static float segmentMCtoEdgeLeft(const int /*segment*/){ return 0.03*SCALE; }
    static float segmentMCtoEdgeRight(const int /*segment*/) { return 0.03f*SCALE; }
    static float segmentMCtoEdgeForward(const int /*segment*/) { return 0.045f*SCALE; }
    static float segmentMCtoEdgeBackward(const int /*segment*/) { return 0.045f*SCALE; }
    static float segmentMCtoBackwardJointBegin(const int /*segment*/){ return 0.042f*SCALE; }
    static float segmentMCtoBackwardJointEnd(const int /*segment*/){ return 0.052f*SCALE; }
    static float segmentMCtoForwardJointBegin(const int /*segment*/){ return 0.042f*SCALE; }
    static float segmentMCtoForwardJointEnd(const int /*segment*/){ return 0.052f*SCALE; }
    static float segmentMCtoBackwardJointConnection(const int /*segment*/) { return 0.05f*SCALE; }
    static float segmentMCtoForwardJointConnection(const int /*segment*/) { return 0.05f*SCALE; }
    static float jointForwardWidth(const int /*segment*/){ return 0.02f*SCALE; }
    static float jointBackwardWidth(const int /*segment*/){ return 0.02f*SCALE; }
};

template <unsigned int SCALE>
class CenterOfMass_dimensions
{
public:
    static float circleRadius(const int /*segment*/){ return 0.01f*SCALE; }
};

// Arrow to show directions and magnitude of force and speed vectors. Note that the below arrow length is when the measurable is of unity
// Arrow length is total length of the arrow, including the arrowhead.
template <unsigned int SCALE>
class Arrow_dimensions
{
public:
    static float arrowBreadth(){ return 0.01f*SCALE; }
    static float arrowLength(){ return 0.15f*SCALE; }
    static float arrowHeadBreadth(){ return 0.03f*SCALE; }
    static float arrowHeadLength(){ return 0.06f*SCALE; }
};

#endif // DIMENSIONS_H

