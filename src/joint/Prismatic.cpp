#include "joint/Prismatic.h"
#include "rigidbody/RigidBody.h"

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

Prismatic::Prismatic() : Joint()
{

}

Prismatic::Prismatic(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1) : 
    Joint(_body0, _body1, _r0, _q0, _r1, _q1)
{
    dim = 5;
    J0.setZero(5, 6);
    J1.setZero(5, 6);
    J0Minv.setZero(5, 6);
    J1Minv.setZero(5, 6);
    phi.setZero(5);
    lambda.setZero(5);
}

void Prismatic::computeJacobian()
{
    static const Eigen::Matrix3f sEye = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;
    const Eigen::Vector3f d01 = body1->x + rr1 - body0->x - rr0;

    const Eigen::Vector3f n = body0->q * (q0 * Eigen::Vector3f(1, 0, 0));
    const Eigen::Vector3f u = body0->q * (q0 * Eigen::Vector3f(0, 1, 0));
    const Eigen::Vector3f v = body0->q * (q0 * Eigen::Vector3f(0, 0, 1));
    const Eigen::Vector3f nn = body1->q * (q1 * Eigen::Vector3f(0, 1, 0));
    const Eigen::Vector3f uu = body1->q * (q1 * Eigen::Vector3f(0, 1, 0));
    const Eigen::Vector3f vv = body1->q * (q1 * Eigen::Vector3f(0, 0, 1));

    const Eigen::Vector3f ncrossuu = nn.cross(uu);
    const Eigen::Vector3f ncrossvv = nn.cross(vv);

    // compute constraint error
    phi(0) = d01.dot(u);
    phi(1) = d01.dot(v);
    phi(2) = n.dot(uu);
    phi(3) = n.dot(vv);
    phi(4) = u.dot(vv);

    // Compute Jacobian
    // Body 0 //
    J0.block(0, 0, 1, 3) = -u.transpose();
    J0.block(0, 3, 1, 3) = -(rr0 + d01).cross(u).transpose();
    J0.block(1, 0, 1, 3) = -v.transpose();
    J0.block(1, 3, 1, 3) = -(rr0 + d01).cross(v).transpose();
    J0.block(2, 3, 1, 3) = n.cross(uu).transpose();
    J0.block(3, 3, 1, 3) = n.cross(vv).transpose();
    J0.block(4, 3, 1, 3) = u.cross(vv).transpose();
    // Body 1 //
    J1.block(0, 0, 1, 3) = u.transpose();
    J1.block(0, 3, 1, 3) = (rr1).cross(u).transpose();
    J1.block(1, 0, 1, 3) = v.transpose();
    J1.block(1, 3, 1, 3) = (rr1).cross(v).transpose();
    J1.block(2, 3, 1, 3) = -n.cross(uu).transpose();
    J1.block(3, 3, 1, 3) = -n.cross(vv).transpose();
    J1.block(4, 3, 1, 3) = -u.cross(vv).transpose();

    J0Minv.block(0, 0, 5, 3) = (1.0f / body0->mass) * J0.block(0, 0, 5, 3);
    J0Minv.block(0, 3, 5, 3) = J0.block(0, 3, 5, 3) * body0->Iinv;
    J1Minv.block(0, 0, 5, 3) = (1.0f / body1->mass) * J1.block(0, 0, 5, 3);
    J1Minv.block(0, 3, 5, 3) = J1.block(0, 3, 5, 3) * body1->Iinv;
}

