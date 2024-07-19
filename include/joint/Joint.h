#pragma once

#include <Eigen/Dense>

#include "util/Types.h"

class RigidBody;

enum eConstraintType { kContact = 0, kSpherical, kHinge };

// Joint class.
//
class Joint
{
public:

    // Constructor with all parameters.
    Joint(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1);

    Joint(RigidBody* _body0, RigidBody* _body1);

    virtual ~Joint() { }

    RigidBody* body0;           //< The first body
    RigidBody* body1;           //< The second body
    JBlock J0;                  //< The Jacobian of body0
    JBlock J1;                  //< The Jacobian of body1
    JBlock J0Minv;
    JBlock J1Minv;
    Eigen::VectorXf phi;        //< Contraint error
    Eigen::VectorXf lambda;     //< Constraint impulse
    
    unsigned int idx;           //< Used for solver indexing.
    unsigned int dim;           //< Number of constraint equations.

    Eigen::Vector3f r0;         // Relative attachment point of joint in body0 coordinate frame.
    Eigen::Vector3f r1;         // Relative attachment point of joint in body1 coordinate frame.
    Eigen::Quaternionf q0;      // Relative attachment orientation in body0 coordinate frame.
    Eigen::Quaternionf q1;      // Relative attachment orientation in body1 coordinate frame.

    virtual eConstraintType getType() const = 0;

    virtual void computeJacobian() = 0;

protected:

    // Default constructor (hidden).
    Joint();

};
