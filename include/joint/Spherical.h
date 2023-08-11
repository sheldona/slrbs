#pragma once

#include "joint/Joint.h"

// Spherical joint class.
//
class Spherical : public Joint
{
public:

    // Constructor with all parameters.
    Spherical(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1);

    virtual eConstraintType getType() const { return kSpherical; }

    virtual void computeJacobian() override;

    Eigen::Vector3f r0;
    Eigen::Vector3f r1;

protected:
    // Default constructor (hidden).
    Spherical();

};
