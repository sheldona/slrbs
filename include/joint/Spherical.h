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
    virtual void computeGeometricStiffness() override;

    Eigen::Vector3f r0;                 // Relative attachment point of hinge in body0 coordinate frame.
    Eigen::Vector3f r1;                 // Relative attachment point of hinge in body1 coordinate frame.

protected:
    // Default constructor (hidden).
    Spherical();

};
