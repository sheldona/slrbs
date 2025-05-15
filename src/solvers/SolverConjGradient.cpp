#include "solvers/SolverConjGradient.h"

#include "contact/Contact.h"
#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>
#include <limits>
#include <cmath>
#include <algorithm>

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace
{
    static inline void multAndSub(const JBlock &G, const Eigen::Vector3f &x, const Eigen::Vector3f &y,
                                  float a, Eigen::VectorBlock<Eigen::VectorXf> &b)
    {
        b -= a * G.col(0) * x(0);
        b -= a * G.col(1) * x(1);
        b -= a * G.col(2) * x(2);
        b -= a * G.col(3) * y(0);
        b -= a * G.col(4) * y(1);
        b -= a * G.col(5) * y(2);
    }

    static inline void buildRHS(const std::vector<Joint *> &joints, const std::vector<Contact *> &contacts,
                               float h, Eigen::VectorXf &b, bool useOpenMP)
    {
        const float hinv  = 1.0f / h;
        const float gamma = 0.3f;

        // Build RHS for joints
#ifdef USE_OPENMP
        if (useOpenMP && joints.size() > 16)
        {
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(joints.size()); ++i)
            {
                Joint *j = joints[i];
                b.segment(j->idx, j->dim) = -hinv * gamma * j->phi;

                if (!j->body0->fixed)
                {
                    auto seg = b.segment(j->idx, j->dim);
                    multAndSub(j->J0Minv, j->body0->f, j->body0->tau, h, seg);
                    multAndSub(j->J0, j->body0->xdot, j->body0->omega, 1.0f, seg);
                }
                if (!j->body1->fixed)
                {
                    auto seg = b.segment(j->idx, j->dim);
                    multAndSub(j->J1Minv, j->body1->f, j->body1->tau, h, seg);
                    multAndSub(j->J1, j->body1->xdot, j->body1->omega, 1.0f, seg);
                }
            }

            // Build RHS for contacts
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(contacts.size()); ++i)
            {
                Contact *c = contacts[i];
                b.segment(c->idx, 3) = -hinv * gamma * c->phi;

                if (!c->body0->fixed)
                {
                    auto seg = b.segment(c->idx, 3);
                    multAndSub(c->J0Minv, c->body0->f, c->body0->tau, h, seg);
                    multAndSub(c->J0, c->body0->xdot, c->body0->omega, 1.0f, seg);
                }
                if (!c->body1->fixed)
                {
                    auto seg = b.segment(c->idx, 3);
                    multAndSub(c->J1Minv, c->body1->f, c->body1->tau, h, seg);
                    multAndSub(c->J1, c->body1->xdot, c->body1->omega, 1.0f, seg);
                }
            }
        }
        else
#endif
        {
            // Serial version
            for (Joint *j : joints)
            {
                b.segment(j->idx, j->dim) = -hinv * gamma * j->phi;

                if (!j->body0->fixed)
                {
                    auto seg = b.segment(j->idx, j->dim);
                    multAndSub(j->J0Minv, j->body0->f, j->body0->tau, h, seg);
                    multAndSub(j->J0, j->body0->xdot, j->body0->omega, 1.0f, seg);
                }
                if (!j->body1->fixed)
                {
                    auto seg = b.segment(j->idx, j->dim);
                    multAndSub(j->J1Minv, j->body1->f, j->body1->tau, h, seg);
                    multAndSub(j->J1, j->body1->xdot, j->body1->omega, 1.0f, seg);
                }
            }

            // Build RHS for contacts
            for (Contact *c : contacts)
            {
                b.segment(c->idx, 3) = -hinv * gamma * c->phi;

                if (!c->body0->fixed)
                {
                    auto seg = b.segment(c->idx, 3);
                    multAndSub(c->J0Minv, c->body0->f, c->body0->tau, h, seg);
                    multAndSub(c->J0, c->body0->xdot, c->body0->omega, 1.0f, seg);
                }
                if (!c->body1->fixed)
                {
                    auto seg = b.segment(c->idx, 3);
                    multAndSub(c->J1Minv, c->body1->f, c->body1->tau, h, seg);
                    multAndSub(c->J1, c->body1->xdot, c->body1->omega, 1.0f, seg);
                }
            }
        }
    }

    static inline void accumulateCoupled(const Joint *j, const JBlock &JMinv, const RigidBody *body,
                                         const Eigen::VectorXf &x, Eigen::VectorXf &Ax)
    {
        // Accumulate coupled joints
        for (Joint *jj : body->joints)
        {
            if (jj == j) continue;
            const auto segOther = x.segment(jj->idx, jj->dim);
            Ax.segment(j->idx, j->dim) += JMinv * ((body == jj->body0 ? jj->J0 : jj->J1).transpose() * segOther);
        }

        // Accumulate coupled contacts
        for (Contact *c : body->contacts)
        {
            const auto segOther = x.segment(c->idx, 3);
            Ax.segment(j->idx, j->dim) += JMinv * ((body == c->body0 ? c->J0 : c->J1).transpose() * segOther);
        }
    }

    static inline void accumulateCoupledForContact(const Contact *c, const JBlock &JMinv, const RigidBody *body,
                                                  const Eigen::VectorXf &x, Eigen::VectorXf &Ax)
    {
        // Accumulate coupled joints
        for (Joint *j : body->joints)
        {
            const auto segOther = x.segment(j->idx, j->dim);
            Ax.segment(c->idx, 3) += JMinv * ((body == j->body0 ? j->J0 : j->J1).transpose() * segOther);
        }

        // Accumulate coupled contacts
        for (Contact *cc : body->contacts)
        {
            if (cc == c) continue;
            const auto segOther = x.segment(cc->idx, 3);
            Ax.segment(c->idx, 3) += JMinv * ((body == cc->body0 ? cc->J0 : cc->J1).transpose() * segOther);
        }
    }

    static inline void computeAx(const std::vector<Joint *> &joints, const std::vector<Contact *> &contacts,
                                const Eigen::VectorXf &x, Eigen::VectorXf &Ax, bool useOpenMP)
    {
        constexpr float eps = 1e-9f;   // to keep A positive‑definite
        Ax.setZero();

#ifdef USE_OPENMP
        if (useOpenMP && (joints.size() > 16 || contacts.size() > 16))
        {
            // Compute Ax for joints with OpenMP
            #pragma omp parallel
            {
                // Thread-local copy of Ax to avoid race conditions
                Eigen::VectorXf localAx = Eigen::VectorXf::Zero(Ax.size());

                #pragma omp for
                for (int i = 0; i < static_cast<int>(joints.size()); ++i)
                {
                    Joint *j = joints[i];
                    localAx.segment(j->idx, j->dim).noalias() += eps * x.segment(j->idx, j->dim);

                    const RigidBody *body0 = j->body0;
                    const RigidBody *body1 = j->body1;

                    if (!body0->fixed)
                    {
                        localAx.segment(j->idx, j->dim) += j->J0Minv * (j->J0.transpose() * x.segment(j->idx, j->dim));
                        accumulateCoupled(j, j->J0Minv, body0, x, localAx);
                    }
                    if (!body1->fixed)
                    {
                        localAx.segment(j->idx, j->dim) += j->J1Minv * (j->J1.transpose() * x.segment(j->idx, j->dim));
                        accumulateCoupled(j, j->J1Minv, body1, x, localAx);
                    }
                }

                // Compute Ax for contacts with OpenMP
                #pragma omp for
                for (int i = 0; i < static_cast<int>(contacts.size()); ++i)
                {
                    Contact *c = contacts[i];
                    localAx.segment(c->idx, 3).noalias() += eps * x.segment(c->idx, 3);

                    const RigidBody *body0 = c->body0;
                    const RigidBody *body1 = c->body1;

                    if (!body0->fixed)
                    {
                        localAx.segment(c->idx, 3) += c->J0Minv * (c->J0.transpose() * x.segment(c->idx, 3));
                        accumulateCoupledForContact(c, c->J0Minv, body0, x, localAx);
                    }
                    if (!body1->fixed)
                    {
                        localAx.segment(c->idx, 3) += c->J1Minv * (c->J1.transpose() * x.segment(c->idx, 3));
                        accumulateCoupledForContact(c, c->J1Minv, body1, x, localAx);
                    }
                }

                // Merge thread-local results
                #pragma omp critical
                {
                    Ax += localAx;
                }
            }
        }
        else
#endif
        {
            // Serial version
            // Compute Ax for joints
            for (Joint *j : joints)
            {
                Ax.segment(j->idx, j->dim).noalias() += eps * x.segment(j->idx, j->dim);

                const RigidBody *body0 = j->body0;
                const RigidBody *body1 = j->body1;

                if (!body0->fixed)
                {
                    Ax.segment(j->idx, j->dim) += j->J0Minv * (j->J0.transpose() * x.segment(j->idx, j->dim));
                    accumulateCoupled(j, j->J0Minv, body0, x, Ax);
                }
                if (!body1->fixed)
                {
                    Ax.segment(j->idx, j->dim) += j->J1Minv * (j->J1.transpose() * x.segment(j->idx, j->dim));
                    accumulateCoupled(j, j->J1Minv, body1, x, Ax);
                }
            }

            // Compute Ax for contacts
            for (Contact *c : contacts)
            {
                Ax.segment(c->idx, 3).noalias() += eps * x.segment(c->idx, 3);

                const RigidBody *body0 = c->body0;
                const RigidBody *body1 = c->body1;

                if (!body0->fixed)
                {
                    Ax.segment(c->idx, 3) += c->J0Minv * (c->J0.transpose() * x.segment(c->idx, 3));
                    accumulateCoupledForContact(c, c->J0Minv, body0, x, Ax);
                }
                if (!body1->fixed)
                {
                    Ax.segment(c->idx, 3) += c->J1Minv * (c->J1.transpose() * x.segment(c->idx, 3));
                    accumulateCoupledForContact(c, c->J1Minv, body1, x, Ax);
                }
            }
        }
    }

    // Project contact constraints to satisfy friction cone
    static inline void projectContactConstraints(const std::vector<Contact *> &contacts, Eigen::VectorXf &x, bool useOpenMP)
    {
        const float eps = 1e-10f; // Small epsilon to avoid numerical issues with very small values

#ifdef USE_OPENMP
        if (useOpenMP && contacts.size() > 16)
        {
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(contacts.size()); ++i)
            {
                Contact *c = contacts[i];
                // Normal impulse is projected to [0, inf]
                x(c->idx) = std::max(0.0f, x(c->idx));

                // Get normal impulse value for friction cone
                const float normalImpulse = x(c->idx);

                // If normal impulse is very small, set friction to zero to avoid numerical issues
                if (normalImpulse < eps)
                {
                    x(c->idx + 1) = 0.0f;
                    x(c->idx + 2) = 0.0f;
                }
                else
                {
                    // Friction impulses are projected to [-mu * normalImpulse, mu * normalImpulse]
                    const float upperBound = c->mu * normalImpulse;
                    const float lowerBound = -upperBound;

                    x(c->idx + 1) = std::max(lowerBound, std::min(upperBound, x(c->idx + 1)));
                    x(c->idx + 2) = std::max(lowerBound, std::min(upperBound, x(c->idx + 2)));

                    // Additional step for box scenarios: handle tangential friction with a circular constraint
                    float frictionMag = std::sqrt(x(c->idx + 1) * x(c->idx + 1) + x(c->idx + 2) * x(c->idx + 2));
                    if (frictionMag > upperBound && frictionMag > eps) {
                        float scale = upperBound / frictionMag;
                        x(c->idx + 1) *= scale;
                        x(c->idx + 2) *= scale;
                    }
                }
            }
        }
        else
#endif
        {
            // Serial version
            for (Contact *c : contacts)
            {
                // Normal impulse is projected to [0, inf]
                x(c->idx) = std::max(0.0f, x(c->idx));

                // Get normal impulse value for friction cone
                const float normalImpulse = x(c->idx);

                // If normal impulse is very small, set friction to zero to avoid numerical issues
                if (normalImpulse < eps)
                {
                    x(c->idx + 1) = 0.0f;
                    x(c->idx + 2) = 0.0f;
                }
                else
                {
                    // Friction impulses are projected to [-mu * normalImpulse, mu * normalImpulse]
                    const float upperBound = c->mu * normalImpulse;
                    const float lowerBound = -upperBound;

                    x(c->idx + 1) = std::max(lowerBound, std::min(upperBound, x(c->idx + 1)));
                    x(c->idx + 2) = std::max(lowerBound, std::min(upperBound, x(c->idx + 2)));

                    // Additional step for box scenarios: handle tangential friction with a circular constraint
                    float frictionMag = std::sqrt(x(c->idx + 1) * x(c->idx + 1) + x(c->idx + 2) * x(c->idx + 2));
                    if (frictionMag > upperBound && frictionMag > eps) {
                        float scale = upperBound / frictionMag;
                        x(c->idx + 1) *= scale;
                        x(c->idx + 2) *= scale;
                    }
                }
            }
        }
    }
} // anonymous namespace

SolverConjGradient::SolverConjGradient(RigidBodySystem *system) : Solver(system) {}

void SolverConjGradient::solve(float h)
{
    const auto &joints = m_rigidBodySystem->getJoints();
    const auto &contacts = m_rigidBodySystem->getContacts();

    // Map each joint and contact into the big vector
    unsigned int idx = 0;
    for (Joint *j : joints) { j->idx = idx; idx += j->dim; }
    for (Contact *c : contacts) { c->idx = idx; idx += 3; /* 3 DOFs per contact */ }

    // Early exit if no constraints
    if (idx == 0) return;

    Eigen::VectorXf x(idx), r(idx), p(idx), b(idx), Ax(idx);

    x.setZero();
    buildRHS(joints, contacts, h, b, m_useOpenMP);

    // Initial residual setup
    computeAx(joints, contacts, x, Ax, m_useOpenMP);
    r = b - Ax;
    p = r;

    float rsold = r.dot(r);
    float rsnew = rsold;
    float initialResidual = rsold;

    // Early exit if initial residual is already small
    const float tolerance = 1e-8f * initialResidual;
    if (rsold < tolerance) {
        // Store zero solution
#ifdef USE_OPENMP
        if (m_useOpenMP)
        {
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(joints.size()); ++i)
                joints[i]->lambda = x.segment(joints[i]->idx, joints[i]->dim);

            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(contacts.size()); ++i)
                contacts[i]->lambda = x.segment(contacts[i]->idx, 3);
        }
        else
#endif
        {
            for (Joint *j : joints)
                j->lambda = x.segment(j->idx, j->dim);

            for (Contact *c : contacts)
                c->lambda = x.segment(c->idx, 3);
        }
        return;
    }

    // This regularization factor can be used to help stabilize the solver for larger contact sets
    // We'll comment it out since it's not used currently but might be useful in the future
    // const float regFactor = 1.0f + 1e-4f * std::min(static_cast<float>(contacts.size()), 100.0f);

    for (int iter = 0; iter < m_maxIter && rsnew > tolerance; ++iter)
    {
        computeAx(joints, contacts, p, Ax, m_useOpenMP);
        float alpha = rsold / (p.dot(Ax));

        x = x + alpha * p;

        // Project contact constraints
        projectContactConstraints(contacts, x, m_useOpenMP);

        // Recompute residual directly for better numerical stability
        computeAx(joints, contacts, x, Ax, m_useOpenMP);
        r = b - Ax;

        rsnew = r.dot(r);

        float beta = rsnew / rsold;
        p = r + beta * p;
        rsold = rsnew;
    }

    // Store the solution in the joint and contact lambdas
#ifdef USE_OPENMP
    if (m_useOpenMP)
    {
        #pragma omp parallel for
        for (int i = 0; i < static_cast<int>(joints.size()); ++i)
            joints[i]->lambda = x.segment(joints[i]->idx, joints[i]->dim);

        #pragma omp parallel for
        for (int i = 0; i < static_cast<int>(contacts.size()); ++i)
            contacts[i]->lambda = x.segment(contacts[i]->idx, 3);
    }
    else
#endif
    {
        for (Joint *j : joints)
            j->lambda = x.segment(j->idx, j->dim);

        for (Contact *c : contacts)
            c->lambda = x.segment(c->idx, 3);
    }
}