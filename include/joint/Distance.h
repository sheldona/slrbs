#pragma once

#include "joint/Joint.h"

// Distance joint class
//
class Distance : public Joint
{
public:

    // Constructor with all parameters.
    Distance(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, const float _l0);

    virtual eConstraintType getType() const override { return kDistance; }

    virtual void computeJacobian() override;

protected:
    // Default constructor (hidden).
    Distance();

    float l0;

};
