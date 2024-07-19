#pragma once

#include "joint/Joint.h"

// Spherical joint class (aka ball-and-socket).
//
class Spherical : public Joint
{
public:

    // Constructor with all parameters.
    Spherical(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1);

    virtual eConstraintType getType() const override { return kSpherical; }

    virtual void computeJacobian() override;

protected:
    // Default constructor (hidden).
    Spherical();

};
