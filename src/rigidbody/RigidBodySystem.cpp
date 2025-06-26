#include "rigidbody/RigidBodySystem.h"

#include "collision/CollisionDetect.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"

#include "solvers/SolverBoxPGS.h"
#include "solvers/SolverConjGradient.h"
#include "solvers/SolverConjResidual.h"

#include <random>

namespace
{
    // 0 = PGS
    // 1 = Conjugate Gradient
    // 2 = Conjugate residual
    //
    static Solver* s_solvers[3] = { nullptr, nullptr, nullptr };

    // Function to generate a random variable with mean mu and variance sigma_squared
    static inline float stochasticFriction(float mu, float sigma_squared) 
    {
        if (sigma_squared < 1e-5f)
            return mu;

        // Create a random device and a Mersenne Twister engine
        std::random_device rd;
        std::mt19937 gen(rd());

        // Standard deviation is the square root of variance
        const float sigma = std::sqrt(sigma_squared);

        // Create a normal distribution with mean mu and standard deviation sigma
        std::normal_distribution<> dist(mu, sigma);

        // Generate and return a random number
        return dist(gen);
    }

    static inline const ContactState* contactMatching(const std::vector<ContactState>& _contactStates, const Contact& _contact)
    {
        const ContactState* ret = nullptr;
        float minErr = FLT_MAX;

        for (const ContactState& cs : _contactStates)
        {
            // TEST - method from "Stabilization for Interactive Rigid Body Contact Force Computations"
            // 
            const int index0 = _contact.body0 == cs.body[0] ? 0 : (_contact.body0 == cs.body[1] ? 1 : -1);
            const int index1 = _contact.body1 == cs.body[0] ? 0 : (_contact.body1 == cs.body[1] ? 1 : -1);

            if (index0 > -1 && index1 > -1)
            {
                const Eigen::Vector3f p0 = cs.body[index0]->q * cs.p[index0] + cs.body[index0]->x;
                const Eigen::Vector3f p1 = cs.body[index1]->q * cs.p[index1] + cs.body[index1]->x;
                const Eigen::Vector3f n0 = cs.body[index0]->q * cs.n[index0];
                const Eigen::Vector3f n1 = cs.body[index1]->q * cs.n[index1];
                const Eigen::Vector3f& n = _contact.n;
                const Eigen::Vector3f s0 = p0 - n.dot(p0) * n;
                const Eigen::Vector3f s1 = p1 - n.dot(p1) * n;
                const float e = (s1 - s0).norm();

                if (e < 1e-1f && e < minErr )
                {
                    minErr = e;
                    ret = &cs;
                }
            }

        }

        return ret;
    }

}

RigidBodySystem::RigidBodySystem() :
    solverIter(10), 
    solverId(0), 
    m_preStepFunc(nullptr), 
    m_resetFunc(nullptr), 
    m_collisionsEnabled(true),
    m_contactState()
{
    m_collisionDetect = std::make_unique<CollisionDetect>(this);
    s_solvers[0] = new SolverBoxPGS(this);
    s_solvers[1] = new SolverConjGradient(this);
    s_solvers[2] = new SolverConjResidual(this);
}

RigidBodySystem::~RigidBodySystem()
{
    clear();
}

void RigidBodySystem::addBody(RigidBody *_b)
{
    m_bodies.push_back(_b);
}

void RigidBodySystem::addJoint(Joint* _j)
{
    m_joints.push_back(_j);

    if (_j->body0) _j->body0->joints.push_back(_j);
    if (_j->body1) _j->body1->joints.push_back(_j);
}

void RigidBodySystem::step(float dt)
{
    // Initialize the system.
    // Apply gravitional forces and reset angular forces.
    // Cleanup contacts from the previous time step.
    //
    for(auto b : m_bodies)
    {
        b->f = b->mass * Eigen::Vector3f(0.f, -9.81f, 0.f);
        b->tau.setZero();
        b->fc.setZero();
        b->tauc.setZero();
        b->contacts.clear();
    }

    // Standard simulation pipeline.
    computeInertias();

    if( m_preStepFunc )
    {
        m_preStepFunc(m_bodies);
    }

    m_collisionDetect->clear();

    if ( m_collisionsEnabled )
    {
        m_collisionDetect->detectCollisions();
        m_collisionDetect->computeContactJacobians();

        for (Contact& c : m_collisionDetect->getContacts())
        {
            c.matched = false;
            const ContactState* cs = contactMatching(m_contactState, c);
            if (cs != nullptr)
            {
                c.matched = true;

                if (cs->state == Contact::kStick)
                {
                    c.mu = cs->mu;
                }
                else
                {
                    c.mu = stochasticFriction(Contact::mu_r, Contact::sigma_r * Contact::sigma_r);
                }
            }
        }
    }

    for (auto j : m_joints)
    {
        j->computeJacobian();
    }

    for(auto b : m_bodies)
    {
        b->fc.setZero();
        b->tauc.setZero();
    }

    calcConstraintForces(dt);


    // Contact caching
    //
    m_contactState.clear();
    if (m_collisionsEnabled)
    {

        for (Contact& c : m_collisionDetect->getContacts())
        {
            ContactState cs;
            cs.mu = c.mu;
            cs.body[0] = c.body0;
            cs.body[1] = c.body1;
            cs.p[0] = c.body0->q.inverse() * (c.p - c.body0->x);
            cs.p[1] = c.body1->q.inverse() * (c.p - c.body1->x);
            cs.n[0] = c.body0->q.inverse() * c.n;
            cs.n[1] = c.body1->q.inverse() * c.n;
            cs.pene = c.pene;
            cs.state = c.state;
            memcpy(cs.lambda, c.lambda.data(), sizeof(cs.lambda));
            m_contactState.push_back(cs);
        }
    }

    // Perform numerical integration to first update the velocities of each rigid body in @a m_bodies, 
    // followed by the positions and orientations.
    //
    for(RigidBody* b : m_bodies)
    {
        if( !b->fixed )
        {
            // Velocity update
            b->xdot += dt * (1.0f/b->mass) * (b->f + b->fc);
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            // Position and orientation update.
            b->x += dt * b->xdot;
            b->q = b->q + 0.5f * dt * Eigen::Quaternionf(0, b->omega[0], b->omega[1], b->omega[2]) * b->q;
            b->q.normalize();
        }
        else
        {
            // Fixed bodies are pinned in place
            // Do not update their position and orientation, and set the velocity to zero.
            b->xdot.setZero();
            b->omega.setZero();
        }
    }

}

void RigidBodySystem::clear()
{
    if( m_resetFunc )
    {
        m_resetFunc();
    }

    m_collisionDetect->clear();
    m_contactState.clear();

    // Cleanup all the joints.
    for (auto j : m_joints)
    {
        delete j;
    }
    m_joints.clear();

    // Finally, cleanup rigid bodies.
    for(auto b : m_bodies)
    {
        delete b;
    }
    m_bodies.clear();

}

void RigidBodySystem::computeInertias()
{
    for(RigidBody* b : m_bodies)
    {
        b->updateInertiaMatrix();
    }
}

// Accessors for the contact constraint array.
const std::vector<Contact>& RigidBodySystem::getContacts() const
{
    return m_collisionDetect->getContacts();
}

std::vector<Contact>& RigidBodySystem::getContacts()
{
    return m_collisionDetect->getContacts();
}

const std::vector<Joint*>& RigidBodySystem::getJoints() const
{
    return m_joints;
}

std::vector<Joint*>& RigidBodySystem::getJoints()
{
    return m_joints;
}

void RigidBodySystem::calcConstraintForces(float dt)
{
    // Solve for the constraint forces lambda
    //
    s_solvers[solverId]->setMaxIter(solverIter);
    s_solvers[solverId]->solve(dt);

    for (const auto j : m_joints)
    {
        const Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
        const Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;
        j->body0->fc += f0.head<3>();
        j->body0->tauc += f0.tail<3>();
        j->body1->fc += f1.head<3>();
        j->body1->tauc += f1.tail<3>();
    }

    // Apply the constraint forces as forces and torques acting on each body.
    // Essentially, we compute contact forces by computing :
    //
    //       f_contact = J^T * lambda
    //
    // for each contact constraint.
    //
    auto contacts = m_collisionDetect->getContacts();
    for(const auto c : contacts)
    {
        // Convert impulses in c->lambda to forces.
        //
        const Eigen::Vector6f f0 = c.J0.transpose() * c.lambda / dt;
        const Eigen::Vector6f f1 = c.J1.transpose() * c.lambda / dt;

        if (!c.body0->fixed)
        {
            c.body0->fc += f0.head<3>();
            c.body0->tauc += f0.tail<3>();
        }
        if (!c.body1->fixed)
        {
            c.body1->fc += f1.head<3>();
            c.body1->tauc += f1.tail<3>();
        }
    }
}
