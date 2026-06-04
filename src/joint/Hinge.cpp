#include "joint/Hinge.h"
#include "rigidbody/RigidBody.h"

#define _USE_MATH_DEFINES
#include <math.h>

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

Hinge::Hinge() : Joint()
{

}

Hinge::Hinge(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Quaternionf& _q0, const Eigen::Vector3f& _r1, const Eigen::Quaternionf& _q1) : 
    Joint(_body0, _body1, _r0, _q0, _r1, _q1)
{
    dim = 5;
    J0.setZero(dim, 6);
    J1.setZero(dim, 6);
    J0Minv.setZero(dim, 6);
    J1Minv.setZero(dim, 6);
    phi.setZero(dim);
    lambda.setZero(dim);
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

    if (m_controlType == kServo)
    {
        assert(dim == 6);

        Eigen::Quaternionf qRel = q1.conjugate() * body1->q.conjugate() * body0->q * q0;

        if (qRel.w() < 0.0f) 
        {
            qRel.coeffs() = -qRel.coeffs();
        }

        const float currentTheta = 2.0f * std::atan2(qRel.x(), qRel.w());

        // 4. Compute the final constraint error relative to target
        float dTheta = currentTheta - m_theta;
        while (dTheta > M_PI)  dTheta -= 2.0f * M_PI;
        while (dTheta < -M_PI) dTheta += 2.0f * M_PI;

        phi(5) = dTheta;

        J0.block(5, 3, 1, 3) = nn.transpose();
        J1.block(5, 3, 1, 3) = -nn.transpose();

        J0Minv.block(0, 0, 6, 3) = (1.0f / body0->mass) * J0.block(0, 0, 6, 3);
        J0Minv.block(0, 3, 6, 3) = J0.block(0, 3, 6, 3) * body0->Iinv;
        J1Minv.block(0, 0, 6, 3) = (1.0f / body1->mass) * J1.block(0, 0, 6, 3);
        J1Minv.block(0, 3, 6, 3) = J1.block(0, 3, 6, 3) * body1->Iinv;
    }
    else 
    {
        assert(dim == 5);
        J0Minv.block(0, 0, 5, 3) = (1.0f / body0->mass) * J0.block(0, 0, 5, 3);
        J0Minv.block(0, 3, 5, 3) = J0.block(0, 3, 5, 3) * body0->Iinv;
        J1Minv.block(0, 0, 5, 3) = (1.0f / body1->mass) * J1.block(0, 0, 5, 3);
        J1Minv.block(0, 3, 5, 3) = J1.block(0, 3, 5, 3) * body1->Iinv;
    }

}

void Hinge::setControl(eControlType _controlType)
{
    m_controlType = _controlType;

    if ( _controlType == kServo )
    {
        dim = 6;
    }
    else 
    {
        dim = 5;
    }

    J0.setZero(dim, 6);
    J1.setZero(dim, 6);
    J0Minv.setZero(dim, 6);
    J1Minv.setZero(dim, 6);
    phi.setZero(dim);
    lambda.setZero(dim);

}

void Hinge::setTargetAngle(float _theta)
{
    m_theta = _theta;
}
