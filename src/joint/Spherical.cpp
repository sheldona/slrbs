#include "joint/Spherical.h"
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
        m << -a1b1 - a2b2, a(1) * b(0), a(2) * b(0),
			a(0) * b(1), -a0b0 - a2b2, a(2) * b(1),
			a(0) * b(2), a(1) * b(2), -a0b0 - a1b1;
        return m;
    }
}


Spherical::Spherical() : Joint(), r0(), r1()
{

}

Spherical::Spherical(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1) : Joint(_body0, _body1),
    r0(_r0), r1(_r1)
{
    dim = 3;
    J0.setZero(3, 6);
    J1.setZero(3, 6);
    J0Minv.setZero(3, 6);
    J1Minv.setZero(3, 6);
    phi.setZero(3);
    lambda.setZero(3);
}

void Spherical::computeJacobian()
{
    static const Eigen::Matrix3f sEye = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;

    // compute constraint error
    phi = (body0->x + rr0 - body1->x - rr1);

    // Compute Jacobian
    J0.block(0,0,3,3) = sEye;
    J1.block(0,0,3,3) = -sEye;
    J0.block(0,3,3,3) = hat(-rr0);
    J1.block(0,3,3,3) = hat(rr1);
    J0Minv.block(0,0,3,3) = (1.0f/body0->mass) * J0.block(0, 0, 3, 3);
    J0Minv.block(0,3,3,3) = J0.block(0, 3, 3, 3) * body0->Iinv;
    J1Minv.block(0,0,3,3) = (1.0f/body1->mass) * J1.block(0, 0, 3, 3);
    J1Minv.block(0,3,3,3) = J1.block(0, 3, 3, 3) * body1->Iinv;
}

void Spherical::computeGeometricStiffness()
{
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;
    
    const Eigen::Vector3f p0 = rr0 - body0->x;
    const Eigen::Vector3f p1 = rr1 - body1->x;

    G0.setZero();
    G0.block<3, 3>(3, 3) = -prodOfCrossProd(lambda.segment<3>(0), p0);
    G1.setZero();
    G1.block<3, 3>(3, 3) = prodOfCrossProd(lambda.segment<3>(0), p1);
}
