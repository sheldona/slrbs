#pragma once

#include "joint/Joint.h"

// Hinge joint class.
//
class Hinge : public Joint
{
public:

    // Constructor with all parameters.
    Hinge(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1);

    Hinge(RigidBody* _body0, RigidBody* _body1,
          const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0,
          const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1,
          const Eigen::Vector3f& _axis);

    virtual eConstraintType getType() const override { return kHinge; }

    virtual void computeJacobian() override;
    virtual void computeGeometricStiffness();

    std::string getTypeName() const override { return "Hinge"; }
    Eigen::Vector3f axis;

protected:
    // Default constructor (hidden).
    Hinge();

};
