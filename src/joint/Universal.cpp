#include "joint/Universal.h"

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

Universal::Universal() : Joint()
{
}

Universal::Universal(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1)
    : Joint(_body0, _body1, _r0, _q0, _r1, _q1, kUniversal)
{
    dim = 4; // Four constraints for universal joint
    J0.setZero(dim, 6);
    J1.setZero(dim, 6);
    J0Minv.setZero(dim, 6);
    J1Minv.setZero(dim, 6);
    phi.setZero(dim);
    lambda.setZero(dim);
}

void Universal::computeJacobian()
{
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;

    // Constraint error: the two constraints for universal joint
    Eigen::Vector3f diff = (body0->x + rr0) - (body1->x + rr1);
    phi(0) = diff.dot(Eigen::Vector3f::UnitX());
    phi(1) = diff.dot(Eigen::Vector3f::UnitZ());

    // Compute the Jacobian blocks
    J0.block<1, 3>(0, 0) = Eigen::Vector3f::UnitX().transpose();
    J0.block<1, 3>(1, 0) = Eigen::Vector3f::UnitZ().transpose();
    J0.block<1, 3>(0, 3) = (rr0.cross(Eigen::Vector3f::UnitX())).transpose();
    J0.block<1, 3>(1, 3) = (rr0.cross(Eigen::Vector3f::UnitZ())).transpose();

    J1.block<1, 3>(0, 0) = -Eigen::Vector3f::UnitX().transpose();
    J1.block<1, 3>(1, 0) = -Eigen::Vector3f::UnitZ().transpose();
    J1.block<1, 3>(0, 3) = -(rr1.cross(Eigen::Vector3f::UnitX())).transpose();
    J1.block<1, 3>(1, 3) = -(rr1.cross(Eigen::Vector3f::UnitZ())).transpose();

    // Compute the inverse mass Jacobians
    J0Minv.block<2, 3>(0, 0) = (1.0f / body0->mass) * J0.block<2, 3>(0, 0);
    J0Minv.block<2, 3>(0, 3) = J0.block<2, 3>(0, 3) * body0->Iinv;
    J1Minv.block<2, 3>(0, 0) = (1.0f / body1->mass) * J1.block<2, 3>(0, 0);
    J1Minv.block<2, 3>(0, 3) = J1.block<2, 3>(0, 3) * body1->Iinv;
}