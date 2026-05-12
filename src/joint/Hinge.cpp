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
    // 1. Setup local vectors in world space
    const Eigen::Vector3f rr0 = body0->q * r0;
    const Eigen::Vector3f rr1 = body1->q * r1;

    const Eigen::Vector3f ni = body0->q * (q0 * Eigen::Vector3f(1, 0, 0));
    const Eigen::Vector3f uj = body1->q * (q1 * Eigen::Vector3f(0, 1, 0));
    const Eigen::Vector3f vj = body1->q * (q1 * Eigen::Vector3f(0, 0, 1));

    // 2. Initialize the full geometric stiffness components 
    // Assuming G0 and G1 are the diagonal blocks, and you may need G01 for coupling
    G0.setZero(); // 6x6 stiffness for body 0
    G1.setZero(); // 6x6 stiffness for body 1

    // --- Ball and Socket Part (Linear constraints 0, 1, 2) ---
    const Eigen::Vector3f lambda_lin = lambda.segment<3>(0);
    G0.block<3, 3>(3, 3) = prodOfCrossProd(lambda_lin, rr0);
    G1.block<3, 3>(3, 3) = -prodOfCrossProd(lambda_lin, rr1);

    // --- Dot-1 Constraints (Angular constraints 3, 4) ---
    // We implement K = lambda * (u' * n^T + n' * u^T) for both u and v
    auto applyDot1Stiffness = [&](float lam, const Eigen::Vector3f& n, const Eigen::Vector3f& u) {
        // n' for body i (body0) is hat(n)
        // u' for body j (body1) is hat(u)
        Eigen::Matrix3f n_hat = hat(n);
        Eigen::Matrix3f u_hat = hat(u);

        // Based on the paper's low-rank decomposition:
        // G_rot_00 (Body 0 diagonal)
        G0.block<3, 3>(3, 3) += lam * (n_hat * u.dot(n) * n_hat.transpose()); 

        // Body 0 (i) contribution:
        G0.block<3, 3>(3, 3) += lam * (u_hat * n_hat).transpose();

        // Body 1 (j) contribution:
        G1.block<3, 3>(3, 3) += lam * (n_hat * u_hat).transpose();
    };

    applyDot1Stiffness(lambda(3), ni, uj);
    applyDot1Stiffness(lambda(4), ni, vj);
}

//void Hinge::computeGeometricStiffness()
//{
//    const Eigen::Vector3f rr0 = body0->q * r0;
//    const Eigen::Vector3f rr1 = body1->q * r1;
//
//    const Eigen::Vector3f nn = body0->q * (q0 * Eigen::Vector3f(1, 0, 0));
//    const Eigen::Vector3f uu = body1->q * (q1 * Eigen::Vector3f(0, 1, 0));
//    const Eigen::Vector3f vv = body1->q * (q1 * Eigen::Vector3f(0, 0, 1));
//
//    const Eigen::Matrix3f unT = uu * nn.transpose();
//    const Eigen::Matrix3f vnT = vv * nn.transpose();
//
//    G0.setZero();
//    // Positional stiffness (scaled by linear lambdas 0, 1, 2)
//    G0.block<3, 3>(3, 3) += prodOfCrossProd(lambda.segment<3>(0), rr0);
//
//    // Rotational stiffness
//    G0.block<3, 3>(3, 3) += lambda(3)*unT.transpose();
//    G0.block<3, 3>(3, 3) += lambda(4)*vnT.transpose();
//
//    G1.setZero();
//    G1.block<3, 3>(3, 3) += -prodOfCrossProd(lambda.segment<3>(0), rr1);
//    G1.block<3, 3>(3, 3) += lambda(3)*unT;
//    G1.block<3, 3>(3, 3) += lambda(4)*vnT;
//
//}