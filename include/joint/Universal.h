#pragma once

#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Dense>

class Universal : public Joint
{
public:
    // Constructor with all parameters.
    Universal(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1);

    // Override computeJacobian function.
    virtual void computeJacobian() override;

protected:
    // Default constructor (hidden).
    Universal();
};