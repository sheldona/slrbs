#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"

Joint::Joint() :
    body0(nullptr), body1(nullptr), dim(0),
    r0(Eigen::Vector3f::Zero()), r1(Eigen::Vector3f::Zero()), q0(Eigen::Quaternionf::Identity()), q1(Eigen::Quaternionf::Identity())
{

}


Joint::Joint(RigidBody* _body0, RigidBody* _body1) : body0(_body0), body1(_body1), dim(0), 
    r0(Eigen::Vector3f::Zero()), r1(Eigen::Vector3f::Zero()), q0(Eigen::Quaternionf::Identity()), q1(Eigen::Quaternionf::Identity())
{

}

Joint::Joint(RigidBody* _body0, RigidBody* _body1,
    const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1) :
    body0(_body0), body1(_body1), dim(0), r0(_r0), r1(_r1), q0(_q0), q1(_q1)
{
    J0.setZero();
    J1.setZero();
    J0Minv.setZero();
    J1Minv.setZero();
    lambda.setZero();
    phi.setZero();
}
