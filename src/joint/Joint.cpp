#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"

Joint::Joint() :
    body0(nullptr), body1(nullptr), dim(0), idx(0), type(kSpherical),
    r0(Eigen::Vector3f::Zero()), r1(Eigen::Vector3f::Zero()),
    q0(Eigen::Quaternionf::Identity()), q1(Eigen::Quaternionf::Identity())
{
    // Initialize empty matrices and vectors
    J0.setZero();
    J1.setZero();
    J0Minv.setZero();
    J1Minv.setZero();
    G0.setZero();
    G1.setZero();
    phi.setZero();
    lambda.setZero();
}

Joint::Joint(RigidBody* _body0, RigidBody* _body1, eConstraintType _type) :
    body0(_body0), body1(_body1), type(_type), idx(0),
    r0(Eigen::Vector3f::Zero()), r1(Eigen::Vector3f::Zero()),
    q0(Eigen::Quaternionf::Identity()), q1(Eigen::Quaternionf::Identity())
{
    // Initialize dimension based on joint type
    switch (type) {
        case kSpherical: dim = 3; break;
        case kHinge: dim = 5; break;
        case kDistance: dim = 1; break;
        case kPrismatic: dim = 5; break;
        case kUniversal: dim = 4; break;
        case kRigid: dim = 6; break;
        case kFlexible: dim = 6; break;
        default: dim = 3; break;
    }

    // Initialize matrices and vectors to proper dimensions
    J0.setZero(dim, 6);
    J1.setZero(dim, 6);
    J0Minv.setZero(dim, 6);
    J1Minv.setZero(dim, 6);
    G0.setZero(6, 6);
    G1.setZero(6, 6);
    phi.setZero(dim);
    lambda.setZero(dim);
}

Joint::Joint(RigidBody* _body0, RigidBody* _body1,
    const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1,
    eConstraintType _type) :
    body0(_body0), body1(_body1), type(_type), idx(0),
    r0(_r0), r1(_r1),
    q0(Eigen::Quaternionf::Identity()), q1(Eigen::Quaternionf::Identity())
{
    // Initialize dimension based on joint type
    switch (type) {
        case kSpherical: dim = 3; break;
        case kHinge: dim = 5; break;
        case kDistance: dim = 1; break;
        case kPrismatic: dim = 5; break;
        case kUniversal: dim = 4; break;
        case kRigid: dim = 6; break;
        case kFlexible: dim = 6; break;
        default: dim = 3; break;
    }

    // Initialize matrices and vectors to proper dimensions
    J0.setZero(dim, 6);
    J1.setZero(dim, 6);
    J0Minv.setZero(dim, 6);
    J1Minv.setZero(dim, 6);
    G0.setZero(6, 6);
    G1.setZero(6, 6);
    phi.setZero(dim);
    lambda.setZero(dim);
}

Joint::Joint(RigidBody* _body0, RigidBody* _body1,
    const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0,
    const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1,
    eConstraintType _type) :
    body0(_body0), body1(_body1), type(_type), idx(0),
    r0(_r0), r1(_r1), q0(_q0), q1(_q1)
{
    // Initialize dimension based on joint type
    switch (type) {
        case kSpherical: dim = 3; break;
        case kHinge: dim = 5; break;
        case kDistance: dim = 1; break;
        case kPrismatic: dim = 5; break;
        case kUniversal: dim = 4; break;
        case kRigid: dim = 6; break;
        case kFlexible: dim = 6; break;
        default: dim = 3; break;
    }

    // Initialize matrices and vectors to proper dimensions
    J0.setZero(dim, 6);
    J1.setZero(dim, 6);
    J0Minv.setZero(dim, 6);
    J1Minv.setZero(dim, 6);
    G0.setZero(6, 6);
    G1.setZero(6, 6);
    phi.setZero(dim);
    lambda.setZero(dim);
}