#pragma once

#include "joint/Joint.h"

// Distance joint class.
//
class Distance : public Joint
{
public:

    // Constructor with all parameters.
    Distance(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, float _d);

    virtual void computeJacobian() override;

    float d;            // Fixed distance to enforce between attachments r0 on body0 and r1 on body1.

protected:
    // Default constructor (hidden).
    Distance();

};