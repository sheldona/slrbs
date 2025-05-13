#include "solvers/SolverConjResidual.h"

#include "contact/Contact.h"
#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>
#include <limits>

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
                               float h, Eigen::VectorXf &b)
    {
        const float hinv  = 1.0f / h;
        const float gamma = 0.3f;

        // Build RHS for joints
        for (Joint *j : joints)
        {
            b.segment(j->idx, j->dim) = -hinv * gamma * j->phi;

            if (!j->body0->fixed)
            {
                auto seg = b.segment(j->idx, j->dim);
                multAndSub(j->J0Minv, j->body0->f,   j->body0->tau,  h,   seg);
                multAndSub(j->J0,     j->body0->xdot, j->body0->omega, 1.0f, seg);
            }
            if (!j->body1->fixed)
            {
                auto seg = b.segment(j->idx, j->dim);
                multAndSub(j->J1Minv, j->body1->f,   j->body1->tau,  h,   seg);
                multAndSub(j->J1,     j->body1->xdot, j->body1->omega, 1.0f, seg);
            }
        }

        // Build RHS for contacts
        for (Contact *c : contacts)
        {
            b.segment(c->idx, 3) = -hinv * gamma * c->phi;

            if (!c->body0->fixed)
            {
                auto seg = b.segment(c->idx, 3);
                multAndSub(c->J0Minv, c->body0->f,   c->body0->tau,  h,   seg);
                multAndSub(c->J0,     c->body0->xdot, c->body0->omega, 1.0f, seg);
            }
            if (!c->body1->fixed)
            {
                auto seg = b.segment(c->idx, 3);
                multAndSub(c->J1Minv, c->body1->f,   c->body1->tau,  h,   seg);
                multAndSub(c->J1,     c->body1->xdot, c->body1->omega, 1.0f, seg);
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
                                const Eigen::VectorXf &x, Eigen::VectorXf &Ax)
    {
        constexpr float eps = 1e-9f;   // to keep A positive‑definite
        Ax.setZero();

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

    // Project contact constraints to satisfy friction cone
    static inline void projectContactConstraints(const std::vector<Contact *> &contacts, Eigen::VectorXf &x)
    {
        const float eps = 1e-10f; // Small epsilon to avoid numerical issues with very small values

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
            }
        }
    }
} // anonymous namespace

SolverConjResidual::SolverConjResidual(RigidBodySystem *system) : Solver(system) {}

void SolverConjResidual::solve(float h)
{
    const auto &joints = m_rigidBodySystem->getJoints();
    const auto &contacts = m_rigidBodySystem->getContacts();

    // Map each joint and contact into the big vector
    unsigned int idx = 0;
    for (Joint *j : joints) { j->idx = idx; idx += j->dim; }
    for (Contact *c : contacts) { c->idx = idx; idx += 3; /* 3 DOFs per contact */ }

    // Early exit if no constraints
    if (idx == 0) return;

    Eigen::VectorXf x(idx), r(idx), p(idx), b(idx), Ax(idx), Ap(idx), Ar(idx);

    x.setZero();
    buildRHS(joints, contacts, h, b);

    // Initial residual setup
    computeAx(joints, contacts, x, Ax);
    r = b - Ax;
    computeAx(joints, contacts, r, Ar);

    float rAr = r.dot(Ar);
    float rAr0 = rAr;

    // Early exit if initial residual is already small
    const float tolerance = 1e-12f * rAr0;
    if (rAr < tolerance) {
        // Store zero solution
        for (Joint *j : joints)
            j->lambda = x.segment(j->idx, j->dim);

        for (Contact *c : contacts)
            c->lambda = x.segment(c->idx, 3);
        return;
    }

    // Initial search direction setup
    p = r;
    computeAx(joints, contacts, p, Ap);
    float pATAp = Ap.dot(Ap);

    // Avoid division by zero
    if (pATAp < 1e-12f) {
        // Store zero solution if the system is singular
        for (Joint *j : joints)
            j->lambda = x.segment(j->idx, j->dim);

        for (Contact *c : contacts)
            c->lambda = x.segment(c->idx, 3);
        return;
    }

    for (int iter = 0; iter < m_maxIter && rAr > tolerance && pATAp > 1e-12f; ++iter)
    {
        const float alpha = rAr / pATAp;
        x += alpha * p;

        // Project constraints for contacts
        projectContactConstraints(contacts, x);

        // Recompute residual after projection for better numerical stability
        computeAx(joints, contacts, x, Ax);
        r = b - Ax;
        computeAx(joints, contacts, r, Ar);

        const float rArNext = r.dot(Ar);

        const float beta = rArNext / rAr;
        p = r + beta * p;

        // Recompute Ap directly rather than using the update formula
        // This provides better numerical stability
        computeAx(joints, contacts, p, Ap);

        rAr = rArNext;
        pATAp = Ap.dot(Ap);
    }

    // Store the solution in the joint and contact lambdas
    for (Joint *j : joints)
        j->lambda = x.segment(j->idx, j->dim);

    for (Contact *c : contacts)
        c->lambda = x.segment(c->idx, 3);
}