#pragma once

#include "joint/Joint.h"

// Hinge joint class.
//
class Hinge : public Joint
{
public:

    // Constructor with all parameters.
    Hinge(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1);

    virtual eConstraintType getType() const override { return kHinge; }

    virtual void computeJacobian() override;

    Eigen::Vector3f r0;
    Eigen::Vector3f r1;
    Eigen::Quaternionf q0;
    Eigen::Quaternionf q1;

protected:
    // Default constructor (hidden).
    Hinge();

};
