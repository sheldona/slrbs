#include "joint/Hinge.h"
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

    static inline Eigen::Matrix3f prodOfCrossProd(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
    {
        float a0b0 = a(0) * b(0);
        float a1b1 = a(1) * b(1);
        float a2b2 = a(2) * b(2);
        Eigen::Matrix3f m;
        m << -a1b1 - a2b2, a(1)* b(0), a(2)* b(0),
            a(0)* b(1), -a0b0 - a2b2, a(2)* b(1),
            a(0)* b(2), a(1)* b(2), -a0b0 - a1b1;
        return m;
    }
}

Hinge::Hinge() : Joint()
{

}

Hinge::Hinge(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1) : 
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

void Hinge::computeJacobian()
{
    static const Eigen::Matrix3f sEye = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;
    const Eigen::Vector3f nn = body0->q * (q0 * Eigen::Vector3f(1, 0, 0));
    const Eigen::Vector3f uu = body1->q * (q1 * Eigen::Vector3f(0, 1, 0));
    const Eigen::Vector3f vv = body1->q * (q1 * Eigen::Vector3f(0, 0, 1));
    const Eigen::Vector3f ncrossuu = nn.cross(uu);
    const Eigen::Vector3f ncrossvv = nn.cross(vv);

    // compute constraint error
    phi.segment(0, 3) = (body0->x + rr0 - body1->x - rr1);
    phi(3) = nn.dot(uu);
    phi(4) = nn.dot(vv);

    // Compute Jacobian
    J0.block(0, 0, 3, 3) = sEye;
    J0.block(3, 3, 1, 3) = ncrossuu.transpose();
    J0.block(4, 3, 1, 3) = ncrossvv.transpose();
    J0.block(0, 3, 3, 3) = hat(-rr0);
    J1.block(0, 0, 3, 3) = -sEye;
    J1.block(0, 3, 3, 3) = hat(rr1);
    J1.block(3, 3, 1, 3) = -ncrossuu.transpose();
    J1.block(4, 3, 1, 3) = -ncrossvv.transpose();

    J0Minv.block(0, 0, 5, 3) = (1.0f / body0->mass) * J0.block(0, 0, 5, 3);
    J0Minv.block(0, 3, 5, 3) = J0.block(0, 3, 5, 3) * body0->Iinv;
    J1Minv.block(0, 0, 5, 3) = (1.0f / body1->mass) * J1.block(0, 0, 5, 3);
    J1Minv.block(0, 3, 5, 3) = J1.block(0, 3, 5, 3) * body1->Iinv;

}

void Hinge::computeGeometricStiffness()
{
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;

    const Eigen::Vector3f p0 = rr0 - body0->x;
    const Eigen::Vector3f p1 = rr1 - body1->x;
    const Eigen::Vector3f nn = body0->q * (q0 * Eigen::Vector3f(1, 0, 0));
    const Eigen::Vector3f uu = body1->q * (q1 * Eigen::Vector3f(0, 1, 0));
    const Eigen::Vector3f vv = body1->q * (q1 * Eigen::Vector3f(0, 0, 1));

    const Eigen::Matrix3f unT = uu * nn.transpose();
    const Eigen::Matrix3f vnT = vv * nn.transpose();

    G0.setZero();
    G0.block<3, 3>(3, 3) += prodOfCrossProd(lambda.segment<3>(0), p0);
    G0.block<3, 3>(3, 3) += -unT.transpose();
    G0.block<3, 3>(3, 3) += -vnT.transpose();
    // TODO: missing additional off-diagonal blocks here. 
    //  Need to modify G block matrix to account for 12x12 version of geom stiffness matrix



    G1.setZero();
    G1.block<3, 3>(3, 3) += -prodOfCrossProd(lambda.segment<3>(0), p1);
    G1.block<3, 3>(3, 3) += unT.transpose();
    G1.block<3, 3>(3, 3) += vnT.transpose();

}