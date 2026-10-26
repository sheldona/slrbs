#pragma once

#include "joint/Joint.h"

// Spring class
//
class Spring : public Joint
{
public:

    // Constructor with all parameters.
    Spring(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, const float _l0);

    virtual eConstraintType getType() const override { return kSpring; }

    virtual void computeJacobian() override;

protected:
    // Default constructor (hidden).
    Spring();

    float l0;

};
