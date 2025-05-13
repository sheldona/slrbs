#pragma once

#include <Eigen/Dense>
#include "util/Types.h"

class RigidBody;

// Extended enum to include all joint types
enum eConstraintType {
    kContact = 0,
    kSpherical,
    kHinge,
    kDistance,
    kPrismatic,
    kUniversal,
    kRigid,        // For 6D rigid joint
    kFlexible      // For flexible cable joint
};

// Joint class.
//
class Joint
{
public:
    // Constructor with all parameters including orientation
    Joint(RigidBody* _body0, RigidBody* _body1,
          const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0,
          const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1,
          eConstraintType _type = kSpherical);

    // Simplified constructor without orientation
    Joint(RigidBody* _body0, RigidBody* _body1,
          const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1,
          eConstraintType _type = kSpherical);

    // Basic constructor with only bodies
    Joint(RigidBody* _body0, RigidBody* _body1, eConstraintType _type = kSpherical);

    virtual ~Joint() { }

    RigidBody* body0;           //< The first body
    RigidBody* body1;           //< The second body
    JBlock J0;                  //< The Jacobian of body0
    JBlock J1;                  //< The Jacobian of body1
    JBlock J0Minv;              //< J0 * inverse mass matrix of body0
    JBlock J1Minv;              //< J1 * inverse mass matrix of body1
    GBlock G0;                  //< Geometric stiffness of body0
    GBlock G1;                  //< Geometric stiffness of body1
    Eigen::VectorXf phi;        //< Constraint error
    Eigen::VectorXf lambda;     //< Constraint impulse

    unsigned int idx;           //< Used for solver indexing.
    unsigned int dim;           //< Number of constraint equations.

    Eigen::Vector3f r0;         // Relative attachment point of joint in body0 coordinate frame.
    Eigen::Vector3f r1;         // Relative attachment point of joint in body1 coordinate frame.
    Eigen::Quaternionf q0;      // Relative attachment orientation in body0 coordinate frame.
    Eigen::Quaternionf q1;      // Relative attachment orientation in body1 coordinate frame.
    eConstraintType type;       // Type of constraint

    virtual eConstraintType getType() const { return type; }

    virtual void computeJacobian() = 0;
    virtual void computeGeometricStiffness() {}

    virtual std::string getTypeName() const { return "Joint"; }

protected:
    // Default constructor (hidden).
    Joint();
};