#include "joint/Prismatic.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Dense>
#include <iostream>
#include <cassert>

namespace
{
    static inline Eigen::Matrix3f hat(const Eigen::Vector3f& v)
    {
        Eigen::Matrix3f vhat;
        vhat << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return vhat;
    }
}

Prismatic::Prismatic() : Joint(), axis()
{
}

Prismatic::Prismatic(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, const Eigen::Vector3f& _axis)
    : Joint(_body0, _body1, _r0, Eigen::Quaternionf::Identity(), _r1, Eigen::Quaternionf::Identity(), kPrismatic), axis(_axis)
{
    dim = 5; // Five constraints for a prismatic joint
    J0.setZero(dim, 6);
    J1.setZero(dim, 6);
    J0Minv.setZero(dim, 6);
    J1Minv.setZero(dim, 6);
    phi.setZero(dim);
    lambda.setZero(dim);
}


void Prismatic::computeJacobian()
{
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;

    // Constraint error: the constraint along the prismatic axis
    Eigen::Vector3f diff = (body0->x + rr0) - (body1->x + rr1);
    phi(0) = diff.dot(axis);

    // Compute the Jacobian blocks
    J0.block(0, 0, 1, 3) = axis.transpose();
    J0.block(0, 3, 1, 3) = (rr0.cross(axis)).transpose();

    J1.block(0, 0, 1, 3) = -axis.transpose();
    J1.block(0, 3, 1, 3) = -(rr1.cross(axis)).transpose();

    // Compute the inverse mass Jacobians
    J0Minv.block(0, 0, 1, 3) = (1.0f / body0->mass) * J0.block(0, 0, 1, 3);
    J0Minv.block(0, 3, 1, 3) = J0.block(0, 3, 1, 3) * body0->Iinv;
    J1Minv.block(0, 0, 1, 3) = (1.0f / body1->mass) * J1.block(0, 0, 1, 3);
    J1Minv.block(0, 3, 1, 3) = J1.block(0, 3, 1, 3) * body1->Iinv;
}