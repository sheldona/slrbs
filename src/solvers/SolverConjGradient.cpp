#include "solvers/SolverConjGradient.h"

#include "contact/Contact.h"
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

    static inline void computeAx(const std::vector<Joint*>& joints, const Eigen::VectorXf& x, Eigen::VectorXf& Ax)
    {
        Ax.setZero();
        for (Joint* j : joints)
        {
            const RigidBody* body0 = j->body0;
            const RigidBody* body1 = j->body1;

            if (!body0->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J0Minv * (j->J0.transpose() * x.segment(j->idx, j->dim));

                for (Joint* jj : body0->joints)
                {
                    if (jj != j)
                    {
                        const int otherDim = jj->lambda.rows();
                        if (body0 == jj->body0)
                        {
                            Ax.segment(j->idx, j->dim) += j->J0Minv * (jj->J0.transpose() * x.segment(jj->idx, jj->dim));
                        }
                        else
                        {
                            Ax.segment(j->idx, j->dim) += j->J0Minv * (jj->J1.transpose() * x.segment(jj->idx, jj->dim));
                        }
                    }
                }
            }
            if (!body1->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J1Minv * (j->J1.transpose() * x.segment(j->idx, j->dim));

                for (Joint* jj : body1->joints)
                {
                    if (jj != j)
                    {
                        const int otherDim = jj->lambda.rows();
                        if (body1 == jj->body0)
                        {
                            Ax.segment(j->idx, j->dim) += j->J1Minv * (jj->J0.transpose() * x.segment(jj->idx, jj->dim));
                        }
                        else
                        {
                            Ax.segment(j->idx, j->dim) += j->J1Minv * (jj->J1.transpose() * x.segment(jj->idx, jj->dim));
                        }
                    }
                }
            }
        }


    }

}

SolverConjGradient::SolverConjGradient(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{

}

void SolverConjGradient::solve(float h, int substeps)
{
    const auto& bodies = m_rigidBodySystem->getBodies();
    const auto& joints = m_rigidBodySystem->getJoints();
    const unsigned int n_bodies = bodies.size();
    const unsigned int n_joints = bodies.size();
    const unsigned int dim = 6 * n_bodies;

    unsigned int idx = 0;
    for (Joint* j : joints)
    {
        j->idx = idx;
        idx += j->dim;
    }

    Eigen::VectorXf x(idx), r(idx), p(idx), b(idx), Ax(idx), Ap(idx), Ar(idx);

    x.setZero();
    buildRHS(joints, h, b);

    // Compute initial Ax and r
    computeAx(joints, x, Ax);
    r = b - Ax;
    p = r;

    float rTr = (r.dot(r));
    computeAx(joints, p, Ap);
    float pAp = p.dot(Ap);

    for (int i = 0; i < m_maxIter; ++i)
    {       
        if (rTr < 1e-12f) break;
        if (pAp < 1e-12f) break;

        const float alpha = rTr / pAp;

        x += alpha * p;
        r -= alpha * Ap;

        const float rTr_next = r.dot(r);
        const float beta = rTr_next / rTr;
        p = r - beta * p;
        computeAx(joints, p, Ap);
        pAp = p.dot(Ap);

        rTr = rTr_next;
    }

    for (Joint* j : joints)
    {
        j->lambda = x.segment(j->idx, j->dim);
    }

}


