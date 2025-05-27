#include "joint/Spring.h"
#include "rigidbody/RigidBody.h"

namespace
{

}


Spring::Spring() : Joint()
{

}

Spring::Spring(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, const float _l0) :
    Joint(_body0, _body1, _r0, Eigen::Quaternionf::Identity(), _r1, Eigen::Quaternionf::Identity()), l0(_l0)
{
    dim = 1;
    J0.setZero(1, 6);
    J1.setZero(1, 6);
    J0Minv.setZero(1, 6);
    J1Minv.setZero(1, 6);
    phi.setZero(1);
    lambda.setZero(1);
}

void Spring::computeJacobian()
{
    static const Eigen::Matrix3f sEye = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;

    Eigen::Vector3f n = body0->x + rr0 - body1->x - rr1;
    const float l = n.norm();
    if (l < 1e-5f)
        n = { 1.0f, 0.0f, 0.0f };
    else
        n /= l;

    // compute constraint error
    phi(0) = l - l0;

    // Compute Jacobian
    J0.block(0, 0, 1, 3) = n.transpose();
    J0.block(0, 3, 1, 3) = -(n.cross(rr0)).transpose();
    J1.block(0, 0, 1, 3) = -n.transpose();
    J1.block(0, 3, 1, 3) = (n.cross(rr1)).transpose();
    J0Minv.block(0,0,1,3) = (1.0f/body0->mass) * J0.block(0, 0, 1, 3);
    J0Minv.block(0,3,1,3) = J0.block(0, 3, 1, 3) * body0->Iinv;
    J1Minv.block(0,0,1,3) = (1.0f/body1->mass) * J1.block(0, 0, 1, 3);
    J1Minv.block(0,3,1,3) = J1.block(0, 3, 1, 3) * body1->Iinv;
}

