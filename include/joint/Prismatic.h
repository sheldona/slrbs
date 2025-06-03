#pragma once

#include "joint/Joint.h"

// Prismatic joint class.
//
class Prismatic : public Joint
{
public:

    // Constructor with all parameters.
    Prismatic(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1);

    virtual eConstraintType getType() const override { return kPrismatic; }

    virtual void computeJacobian() override;

protected:
    // Default constructor (hidden).
    Prismatic();

};
