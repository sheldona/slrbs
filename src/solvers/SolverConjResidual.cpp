#include "solvers/SolverConjResidual.h"

#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>


namespace
{
    static inline void multAndSub(const JBlock& G, const Eigen::Vector3f& x, const Eigen::Vector3f& y, const float& a, Eigen::VectorBlock<Eigen::VectorXf>& b)
    {
        b -= a * G.col(0) * x(0);
        b -= a * G.col(1) * x(1);
        b -= a * G.col(2) * x(2);
        b -= a * G.col(3) * y(0);
        b -= a * G.col(4) * y(1);
        b -= a * G.col(5) * y(2);
    }

    // Computes the right-hand side vector of the Schur complement system: 
    //      b = -phi/h - J*vel - dt*JMinv*force
    //
    static inline void buildRHS(const std::vector<Joint*>& joints, float h, Eigen::VectorXf& b)
    {
        const float hinv = 1.0f / h;
        const float gamma = 0.3f;

        for (Joint* j : joints)
        {
            b.segment(j->idx, j->dim) = -hinv * gamma * j->phi;

            if (!j->body0->fixed)
            {
                multAndSub(j->J0Minv, j->body0->f, j->body0->tau, h, b.segment(j->idx, j->dim));
                multAndSub(j->J0, j->body0->xdot, j->body0->omega, 1.0f, b.segment(j->idx, j->dim));
            }
            if (!j->body1->fixed)
            {
                multAndSub(j->J1Minv, j->body1->f, j->body1->tau, h, b.segment(j->idx, j->dim));
                multAndSub(j->J1, j->body1->xdot, j->body1->omega, 1.0f, b.segment(j->idx, j->dim));
            }
        }

    }

    // Loop over all other contacts for a body and compute modifications to the rhs vector b: 
    //           x -= (JMinv*Jother^T) * lambda_other
    //
    static inline void accumulateCoupledContactsAndJoints(Joint* j, const JBlock& JMinv, const RigidBody* body, const Eigen::VectorXf& x, Eigen::VectorXf& Ax)
    {
        for (Joint* jj : body->joints)
        {
            if (jj != j)
            {
                if (body == jj->body0)
                {
                    Ax.segment(j->idx, j->dim) += JMinv * (jj->J0.transpose() * x.segment(jj->idx, jj->dim));
                }
                else
                {
                    Ax.segment(j->idx, j->dim) += JMinv * (jj->J1.transpose() * x.segment(jj->idx, jj->dim));
                }
            }
        }
    }

    static inline void computeAx(const std::vector<Joint*>& joints, const Eigen::VectorXf& x, Eigen::VectorXf& Ax)
    {
        static const float eps = 1e-9f;
        Ax.setZero();
        for (Joint* j : joints)
        {
            const RigidBody* body0 = j->body0;
            const RigidBody* body1 = j->body1;

            Ax.segment(j->idx, j->dim) += eps * x.segment(j->idx, j->dim);

            if (!body0->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J0Minv * (j->J0.transpose() * x.segment(j->idx, j->dim));
                accumulateCoupledContactsAndJoints(j, j->J0Minv, body0, x, Ax);
            }

            if (!body1->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J1Minv * (j->J1.transpose() * x.segment(j->idx, j->dim));
                accumulateCoupledContactsAndJoints(j, j->J1Minv, body1, x, Ax);
            }
        }
    }
}

SolverConjResidual::SolverConjResidual(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{

}

void SolverConjResidual::solve(float h, int substeps)
{
    const auto& bodies = m_rigidBodySystem->getBodies();
    const auto& joints = m_rigidBodySystem->getJoints();
    const unsigned int n_bodies = bodies.size();
    const unsigned int n_joints = bodies.size();

    unsigned int idx = 0;
    for (Joint* j : joints)
    {
        j->idx = idx;
        idx += j->dim;
    }

    Eigen::VectorXf x(idx), r(idx), p(idx), b(idx), Ax(idx), Ap(idx), Ar(idx), hi(idx), lo(idx);

    x.setZero();
    hi.setConstant(std::numeric_limits<float>::max());
    lo.setConstant(-std::numeric_limits<float>::max());
    buildRHS(joints, h, b);

    // Compute initial Ax and residual
    computeAx(joints, x, Ax);
    r = b - Ax;
    p = r;

    computeAx(joints, p, Ap);
    computeAx(joints, r, Ar);

    float rAr = (r.dot(Ar));
    float pATAp = Ap.dot(Ap);
    for (int i = 0; i < m_maxIter; ++i)
    {
        if (rAr < 1e-12f) break;
        if (pATAp < 1e-12f) break;

        float alpha = rAr / pATAp;
       
        x += alpha * p;
        r -= alpha * Ap;
        computeAx(joints, r, Ar);
        const float rAr_next = r.dot(Ar);

        const float beta = rAr_next / rAr;
        p = r + beta * p;
        Ap = Ar + beta * Ap;

        rAr = rAr_next;
        pATAp = Ap.dot(Ap);

    }

    // Copy joint impulses
    for (Joint* j : joints)
    {
        j->lambda = x.segment(j->idx, j->dim);
    }

}


