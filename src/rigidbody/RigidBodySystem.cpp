#include "rigidbody/RigidBodySystem.h"

#include <solvers/SolverBoxBPP.h>

#include "collision/CollisionDetect.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "solvers/SolverBoxPGS.h"
#include "solvers/SolverBoxBPP.h"
#include "solvers/SolverConjGradient.h"
#include "solvers/SolverConjResidual.h"
#include "solvers/SolverPGSSM.h"
#include "solvers/SolverProximal.h"

#ifdef USE_OPENMP
# include <omp.h>
#endif

namespace {
    static Solver* s_solvers[6] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

    // Graph coloring algorithm for rigid bodies
    void colorRigidBodies(std::vector<RigidBody*>& bodies,
                          const std::vector<Joint*>& joints,
                          const std::vector<Contact*>& contacts) {
        // Reset all colors
        for (auto b : bodies) {
            b->color = -1; // Uncolored
        }

        // Create adjacency list representation of the constraint graph
        std::vector<std::vector<int>> adjacency(bodies.size());

        // Add edges for joints
        for (const auto& joint : joints) {
            if (!joint->body0 || !joint->body1) continue;

            int idx0 = -1, idx1 = -1;
            for (size_t i = 0; i < bodies.size(); ++i) {
                if (bodies[i] == joint->body0) idx0 = static_cast<int>(i);
                if (bodies[i] == joint->body1) idx1 = static_cast<int>(i);
            }

            if (idx0 >= 0 && idx1 >= 0) {
                adjacency[idx0].push_back(idx1);
                adjacency[idx1].push_back(idx0);
            }
        }

        // Add edges for contacts
        for (const auto& contact : contacts) {
            if (!contact->body0 || !contact->body1) continue;

            int idx0 = -1, idx1 = -1;
            for (size_t i = 0; i < bodies.size(); ++i) {
                if (bodies[i] == contact->body0) idx0 = static_cast<int>(i);
                if (bodies[i] == contact->body1) idx1 = static_cast<int>(i);
            }

            if (idx0 >= 0 && idx1 >= 0) {
                adjacency[idx0].push_back(idx1);
                adjacency[idx1].push_back(idx0);
            }
        }

        // Greedy graph coloring algorithm
        std::vector<bool> available(bodies.size(), true);
        int maxColor = 0;

        for (size_t i = 0; i < bodies.size(); ++i) {
            // Mark colors of adjacent vertices as unavailable
            for (int neighbor : adjacency[i]) {
                if (bodies[neighbor]->color >= 0) {
                    available[bodies[neighbor]->color] = false;
                }
            }

            // Find the first available color
            int color;
            for (color = 0; color < static_cast<int>(bodies.size()); ++color) {
                if (available[color]) break;
            }

            // Assign color to this body
            bodies[i]->color = color;
            maxColor = std::max(maxColor, color);

            // Reset available colors for next iteration
            for (int neighbor : adjacency[i]) {
                if (bodies[neighbor]->color >= 0) {
                    available[bodies[neighbor]->color] = true;
                }
            }
        }

        // Store the number of colors for reference
        for (auto b : bodies) {
            b->numColors = maxColor + 1;
        }
    }
}

RigidBodySystem::RigidBodySystem()
 : m_collisionsEnabled(true)
 , m_gravity(0.0f, -9.81f, 0.0f)
 , m_solverType(SolverType::PGS)
 , m_solverIter(10)
 , m_integrationMethod(IntegrationMethod::EXPLICIT_EULER)
 , m_useGraphColoring(true)
{
    m_collisionDetect = std::make_unique<CollisionDetect>(this);
    s_solvers[0] = new SolverBoxPGS(this);
    s_solvers[1] = new SolverConjGradient(this);
    s_solvers[2] = new SolverConjResidual(this);
    s_solvers[3] = new SolverPGSSM(this);
    s_solvers[4] = new SolverProximal(this);
    s_solvers[5] = new SolverBoxBPP(this);
}

RigidBodySystem::~RigidBodySystem() {
    clear();
    for (auto& sol : s_solvers) {
        delete sol;
        sol = nullptr;
    }
}

void RigidBodySystem::addBody(RigidBody* b) {
    m_bodies.push_back(b);
}
void RigidBodySystem::addJoint(Joint* _j) {
    m_joints.push_back(_j);
    if (_j->body0) _j->body0->joints.push_back(_j);
    if (_j->body1) _j->body1->joints.push_back(_j);
}

void RigidBodySystem::step(float dt)
{
#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for(size_t i = 0; i < m_bodies.size(); ++i) {
        auto b = m_bodies[i];
        b->f    = b->mass * m_gravity;
        b->tau.setZero();
        b->fc.setZero();
        b->tauc.setZero();
        b->contacts.clear();
        b->gsDamp.setZero();
    }

    computeInertias();
    if (m_preStepFunc) m_preStepFunc(*this, dt);

    m_collisionDetect->clear();
    if (m_collisionsEnabled) {
        m_collisionDetect->detectCollisions();
        m_collisionDetect->computeContactJacobians();

        // — geometric-stiffness damping pass ————————————————
        if (m_enableGSDamping) {
            // zero out per-body accumulators
            for (auto b : m_bodies) {
                b->gsSum.setZero();
            }
            // accumulate from joints
            for (auto j : m_joints) {
                j->computeGeometricStiffness();
                j->body0->gsSum += j->G0;
                j->body1->gsSum += j->G1;
            }
            // accumulate from contacts
            auto& ctrs = m_collisionDetect->getContacts();
            for (auto c : ctrs) {
                c->computeGeometricStiffness();
                c->body0->gsSum += c->G0;
                c->body1->gsSum += c->G1;
            }
            // convert to damping and update each body’s inertia
            for (auto b : m_bodies) {
                if (b->fixed) continue;
                // for each rotational axis (3..5) we compute a damping entry
                Eigen::Vector3f alphaDamp;
                for (int i = 0; i < 3; ++i) {
                    float k = b->gsSum.col(i+3).norm();
                    // simple linear rule: α * k
                    alphaDamp[i] = m_gsAlpha * k;
                }
                b->gsDamp = alphaDamp;
                b->updateInertiaMatrix();
            }
        }
    }

    // Apply graph coloring if enabled
    if (m_useGraphColoring) {
        colorRigidBodies(m_bodies, m_joints, m_collisionDetect->getContacts());
    }

#ifdef USE_OPENMP
    // Use graph coloring if enabled, otherwise use standard parallelization
    if (m_useGraphColoring && !m_joints.empty()) {
        int numColors = m_bodies.empty() ? 0 : m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_joints.size(); ++i) {
                Joint* j = m_joints[i];
                // Process joint if one of the bodies has the current color
                bool process = false;
                if (j->body0 && j->body0->color == color) process = true;
                if (j->body1 && j->body1->color == color) process = true;

                if (process) {
                    j->computeJacobian();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_joints.size() > 16)
        for (size_t i = 0; i < m_joints.size(); ++i)
            m_joints[i]->computeJacobian();
    }
#else
    for (size_t i = 0; i < m_joints.size(); ++i)
        m_joints[i]->computeJacobian();
#endif

#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for(size_t i = 0; i < m_bodies.size(); ++i) {
        auto b = m_bodies[i];
        b->fc.setZero();
        b->tauc.setZero();
    }

    calcConstraintForces(dt);

    switch (m_integrationMethod) {
        case IntegrationMethod::EXPLICIT_EULER:
            integrateExplicitEuler(dt);   break;
        case IntegrationMethod::SYMPLECTIC_EULER:
            integrateSymplecticEuler(dt); break;
        case IntegrationMethod::VERLET:
            integrateVerlet(dt);          break;
        case IntegrationMethod::RK4:
            integrateRK4(dt);             break;
        case IntegrationMethod::IMPLICIT_EULER:
            integrateImplicitEuler(dt);   break;
        case IntegrationMethod::NEWTON:
            integrateNewton(dt);          break;
        default:
            integrateExplicitEuler(dt);   break;
    }
}

// ——— Explicit Euler —————————————————————————————————————————————————————
void RigidBodySystem::integrateExplicitEuler(float dt) {
#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                RigidBody* b = m_bodies[i];
                if (b->color == color && !b->fixed) {
                    b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
                    b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                    b->x += dt * b->xdot;

                    Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                    Eigen::Quaternionf qDot = omegaQ * b->q;
                    qDot.coeffs() *= 0.5f;
                    b->q.coeffs() += dt * qDot.coeffs();
                    b->q.normalize();
                } else if (b->fixed) {
                    b->xdot.setZero();
                    b->omega.setZero();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_bodies.size() > 16)
        for (auto b : m_bodies) {
            if (!b->fixed) {
                b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
                b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                b->x += dt * b->xdot;

                Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                Eigen::Quaternionf qDot = omegaQ * b->q;
                qDot.coeffs() *= 0.5f;
                b->q.coeffs() += dt * qDot.coeffs();
                b->q.normalize();
            } else {
                b->xdot.setZero();
                b->omega.setZero();
            }
        }
    }
#else
    for (auto b : m_bodies) {
        if (!b->fixed) {
            b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            b->x += dt * b->xdot;

            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}

// ——— Symplectic Euler with graph coloring ———————————————————————————————
void RigidBodySystem::integrateSymplecticEuler(float dt) {
#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                RigidBody* b = m_bodies[i];
                if (b->color == color && !b->fixed) {
                    b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
                    b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                    b->x += dt * b->xdot;

                    Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                    Eigen::Quaternionf qDot = omegaQ * b->q;
                    qDot.coeffs() *= 0.5f;
                    b->q.coeffs() += dt * qDot.coeffs();
                    b->q.normalize();
                } else if (b->fixed) {
                    b->xdot.setZero();
                    b->omega.setZero();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_bodies.size() > 16)
        for (auto b : m_bodies) {
            if (!b->fixed) {
                b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
                b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                b->x += dt * b->xdot;

                Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                Eigen::Quaternionf qDot = omegaQ * b->q;
                qDot.coeffs() *= 0.5f;
                b->q.coeffs() += dt * qDot.coeffs();
                b->q.normalize();
            } else {
                b->xdot.setZero();
                b->omega.setZero();
            }
        }
    }
#else
    for (auto b : m_bodies) {
        if (!b->fixed) {
            b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            b->x += dt * b->xdot;

            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}

// ——— Verlet Integration ———————————————————————————————————————————————————
void RigidBodySystem::integrateVerlet(float dt) {
#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                RigidBody* b = m_bodies[i];
                if (b->color == color && !b->fixed) {
                    // Store previous position for use in velocity update
                    Eigen::Vector3f prev_x = b->x;

                    // Update position
                    b->x += b->xdot * dt + 0.5f * (b->f + b->fc) * dt * dt / b->mass;

                    // Update velocity using central difference
                    b->xdot = b->xdot + 0.5f * dt * (1.0f/b->mass) * (b->f + b->fc);

                    // Update orientation - similar to explicit Euler but with half-step
                    Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                    Eigen::Quaternionf qDot = omegaQ * b->q;
                    qDot.coeffs() *= 0.5f;
                    b->q.coeffs() += dt * qDot.coeffs();
                    b->q.normalize();

                    // Update angular velocity
                    b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));
                } else if (b->fixed) {
                    b->xdot.setZero();
                    b->omega.setZero();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_bodies.size() > 16)
        for (auto b : m_bodies) {
            if (!b->fixed) {
                // Store previous position for use in velocity update
                Eigen::Vector3f prev_x = b->x;

                // Update position
                b->x += b->xdot * dt + 0.5f * (b->f + b->fc) * dt * dt / b->mass;

                // Update velocity using central difference
                b->xdot = b->xdot + 0.5f * dt * (1.0f/b->mass) * (b->f + b->fc);

                // Update orientation - similar to explicit Euler but with half-step
                Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                Eigen::Quaternionf qDot = omegaQ * b->q;
                qDot.coeffs() *= 0.5f;
                b->q.coeffs() += dt * qDot.coeffs();
                b->q.normalize();

                // Update angular velocity
                b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));
            } else {
                b->xdot.setZero();
                b->omega.setZero();
            }
        }
    }
#else
    for (auto b : m_bodies) {
        if (!b->fixed) {
            // Store previous position for use in velocity update
            Eigen::Vector3f prev_x = b->x;

            // Update position
            b->x += b->xdot * dt + 0.5f * (b->f + b->fc) * dt * dt / b->mass;

            // Update velocity using central difference
            b->xdot = b->xdot + 0.5f * dt * (1.0f/b->mass) * (b->f + b->fc);

            // Update orientation - similar to explicit Euler but with half-step
            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();

            // Update angular velocity
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}

// ——— RK4 Integration ———————————————————————————————————————————————————
void RigidBodySystem::integrateRK4(float dt) {
#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                RigidBody* b = m_bodies[i];
                if (b->color == color && !b->fixed) {
                    // Store initial state
                    Eigen::Vector3f x0 = b->x;
                    Eigen::Vector3f v0 = b->xdot;
                    Eigen::Quaternionf q0 = b->q;
                    Eigen::Vector3f omega0 = b->omega;

                    // Calculate acceleration
                    Eigen::Vector3f a0 = (1.0f/b->mass) * (b->f + b->fc);
                    Eigen::Vector3f alpha0 = b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                    // RK4 stage 1
                    Eigen::Vector3f k1_x = v0;
                    Eigen::Vector3f k1_v = a0;
                    Eigen::Quaternionf omegaQ1(0, omega0.x(), omega0.y(), omega0.z());
                    Eigen::Quaternionf k1_q = (omegaQ1 * q0);
                    k1_q.coeffs() *= 0.5f;
                    Eigen::Vector3f k1_omega = alpha0;

                    // RK4 stage 2 (half step)
                    Eigen::Vector3f x1 = x0 + 0.5f * dt * k1_x;
                    Eigen::Vector3f v1 = v0 + 0.5f * dt * k1_v;
                    Eigen::Quaternionf q1 = q0;
                    q1.coeffs() += 0.5f * dt * k1_q.coeffs();
                    q1.normalize();
                    Eigen::Vector3f omega1 = omega0 + 0.5f * dt * k1_omega;

                    // Recalculate forces at midpoint (simplified - using same forces)
                    Eigen::Vector3f a1 = a0;
                    Eigen::Vector3f alpha1 = alpha0;

                    // RK4 stage 2
                    Eigen::Vector3f k2_x = v1;
                    Eigen::Vector3f k2_v = a1;
                    Eigen::Quaternionf omegaQ2(0, omega1.x(), omega1.y(), omega1.z());
                    Eigen::Quaternionf k2_q = (omegaQ2 * q1);
                    k2_q.coeffs() *= 0.5f;
                    Eigen::Vector3f k2_omega = alpha1;

                    // RK4 stage 3 (half step)
                    Eigen::Vector3f x2 = x0 + 0.5f * dt * k2_x;
                    Eigen::Vector3f v2 = v0 + 0.5f * dt * k2_v;
                    Eigen::Quaternionf q2 = q0;
                    q2.coeffs() += 0.5f * dt * k2_q.coeffs();
                    q2.normalize();
                    Eigen::Vector3f omega2 = omega0 + 0.5f * dt * k2_omega;

                    // Recalculate forces at midpoint (simplified - using same forces)
                    Eigen::Vector3f a2 = a0;
                    Eigen::Vector3f alpha2 = alpha0;

                    // RK4 stage 3
                    Eigen::Vector3f k3_x = v2;
                    Eigen::Vector3f k3_v = a2;
                    Eigen::Quaternionf omegaQ3(0, omega2.x(), omega2.y(), omega2.z());
                    Eigen::Quaternionf k3_q = (omegaQ3 * q2);
                    k3_q.coeffs() *= 0.5f;
                    Eigen::Vector3f k3_omega = alpha2;

                    // RK4 stage 4 (full step)
                    Eigen::Vector3f x3 = x0 + dt * k3_x;
                    Eigen::Vector3f v3 = v0 + dt * k3_v;
                    Eigen::Quaternionf q3 = q0;
                    q3.coeffs() += dt * k3_q.coeffs();
                    q3.normalize();
                    Eigen::Vector3f omega3 = omega0 + dt * k3_omega;

                    // Recalculate forces at endpoint (simplified - using same forces)
                    Eigen::Vector3f a3 = a0;
                    Eigen::Vector3f alpha3 = alpha0;

                    // RK4 stage 4
                    Eigen::Vector3f k4_x = v3;
                    Eigen::Vector3f k4_v = a3;
                    Eigen::Quaternionf omegaQ4(0, omega3.x(), omega3.y(), omega3.z());
                    Eigen::Quaternionf k4_q = (omegaQ4 * q3);
                    k4_q.coeffs() *= 0.5f;
                    Eigen::Vector3f k4_omega = alpha3;

                    // Update state with weighted average
                    b->x = x0 + (dt/6.0f) * (k1_x + 2.0f*k2_x + 2.0f*k3_x + k4_x);
                    b->xdot = v0 + (dt/6.0f) * (k1_v + 2.0f*k2_v + 2.0f*k3_v + k4_v);

                    Eigen::Quaternionf qDot;
                    qDot.coeffs() = (1.0f/6.0f) * (k1_q.coeffs() + 2.0f*k2_q.coeffs() + 2.0f*k3_q.coeffs() + k4_q.coeffs());
                    b->q.coeffs() += dt * qDot.coeffs();
                    b->q.normalize();

                    b->omega = omega0 + (dt/6.0f) * (k1_omega + 2.0f*k2_omega + 2.0f*k3_omega + k4_omega);
                } else if (b->fixed) {
                    b->xdot.setZero();
                    b->omega.setZero();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_bodies.size() > 16)
        for (auto b : m_bodies) {
            if (!b->fixed) {
                // Store initial state
                Eigen::Vector3f x0 = b->x;
                Eigen::Vector3f v0 = b->xdot;
                Eigen::Quaternionf q0 = b->q;
                Eigen::Vector3f omega0 = b->omega;

                // Calculate acceleration
                Eigen::Vector3f a0 = (1.0f/b->mass) * (b->f + b->fc);
                Eigen::Vector3f alpha0 = b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                // RK4 stage 1
                Eigen::Vector3f k1_x = v0;
                Eigen::Vector3f k1_v = a0;
                Eigen::Quaternionf omegaQ1(0, omega0.x(), omega0.y(), omega0.z());
                Eigen::Quaternionf k1_q = (omegaQ1 * q0);
                k1_q.coeffs() *= 0.5f;
                Eigen::Vector3f k1_omega = alpha0;

                // RK4 stage 2 (half step)
                Eigen::Vector3f x1 = x0 + 0.5f * dt * k1_x;
                Eigen::Vector3f v1 = v0 + 0.5f * dt * k1_v;
                Eigen::Quaternionf q1 = q0;
                q1.coeffs() += 0.5f * dt * k1_q.coeffs();
                q1.normalize();
                Eigen::Vector3f omega1 = omega0 + 0.5f * dt * k1_omega;

                // Recalculate forces at midpoint (simplified - using same forces)
                Eigen::Vector3f a1 = a0;
                Eigen::Vector3f alpha1 = alpha0;

                // RK4 stage 2
                Eigen::Vector3f k2_x = v1;
                Eigen::Vector3f k2_v = a1;
                Eigen::Quaternionf omegaQ2(0, omega1.x(), omega1.y(), omega1.z());
                Eigen::Quaternionf k2_q = (omegaQ2 * q1);
                k2_q.coeffs() *= 0.5f;
                Eigen::Vector3f k2_omega = alpha1;

                // RK4 stage 3 (half step)
                Eigen::Vector3f x2 = x0 + 0.5f * dt * k2_x;
                Eigen::Vector3f v2 = v0 + 0.5f * dt * k2_v;
                Eigen::Quaternionf q2 = q0;
                q2.coeffs() += 0.5f * dt * k2_q.coeffs();
                q2.normalize();
                Eigen::Vector3f omega2 = omega0 + 0.5f * dt * k2_omega;

                // Recalculate forces at midpoint (simplified - using same forces)
                Eigen::Vector3f a2 = a0;
                Eigen::Vector3f alpha2 = alpha0;

                // RK4 stage 3
                Eigen::Vector3f k3_x = v2;
                Eigen::Vector3f k3_v = a2;
                Eigen::Quaternionf omegaQ3(0, omega2.x(), omega2.y(), omega2.z());
                Eigen::Quaternionf k3_q = (omegaQ3 * q2);
                k3_q.coeffs() *= 0.5f;
                Eigen::Vector3f k3_omega = alpha2;

                // RK4 stage 4 (full step)
                Eigen::Vector3f x3 = x0 + dt * k3_x;
                Eigen::Vector3f v3 = v0 + dt * k3_v;
                Eigen::Quaternionf q3 = q0;
                q3.coeffs() += dt * k3_q.coeffs();
                q3.normalize();
                Eigen::Vector3f omega3 = omega0 + dt * k3_omega;

                // Recalculate forces at endpoint (simplified - using same forces)
                Eigen::Vector3f a3 = a0;
                Eigen::Vector3f alpha3 = alpha0;

                // RK4 stage 4
                Eigen::Vector3f k4_x = v3;
                Eigen::Vector3f k4_v = a3;
                Eigen::Quaternionf omegaQ4(0, omega3.x(), omega3.y(), omega3.z());
                Eigen::Quaternionf k4_q = (omegaQ4 * q3);
                k4_q.coeffs() *= 0.5f;
                Eigen::Vector3f k4_omega = alpha3;

                // Update state with weighted average
                b->x = x0 + (dt/6.0f) * (k1_x + 2.0f*k2_x + 2.0f*k3_x + k4_x);
                b->xdot = v0 + (dt/6.0f) * (k1_v + 2.0f*k2_v + 2.0f*k3_v + k4_v);

                Eigen::Quaternionf qDot;
                qDot.coeffs() = (1.0f/6.0f) * (k1_q.coeffs() + 2.0f*k2_q.coeffs() + 2.0f*k3_q.coeffs() + k4_q.coeffs());
                b->q.coeffs() += dt * qDot.coeffs();
                b->q.normalize();

                b->omega = omega0 + (dt/6.0f) * (k1_omega + 2.0f*k2_omega + 2.0f*k3_omega + k4_omega);
            } else {
                b->xdot.setZero();
                b->omega.setZero();
            }
        }
    }
#else
    for (auto b : m_bodies) {
        if (!b->fixed) {
            // Store initial state
            Eigen::Vector3f x0 = b->x;
            Eigen::Vector3f v0 = b->xdot;
            Eigen::Quaternionf q0 = b->q;
            Eigen::Vector3f omega0 = b->omega;

            // Calculate acceleration
            Eigen::Vector3f a0 = (1.0f/b->mass) * (b->f + b->fc);
            Eigen::Vector3f alpha0 = b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            // RK4 stage 1
            Eigen::Vector3f k1_x = v0;
            Eigen::Vector3f k1_v = a0;
            Eigen::Quaternionf omegaQ1(0, omega0.x(), omega0.y(), omega0.z());
            Eigen::Quaternionf k1_q = (omegaQ1 * q0);
            k1_q.coeffs() *= 0.5f;
            Eigen::Vector3f k1_omega = alpha0;

            // RK4 stage 2 (half step)
            Eigen::Vector3f x1 = x0 + 0.5f * dt * k1_x;
            Eigen::Vector3f v1 = v0 + 0.5f * dt * k1_v;
            Eigen::Quaternionf q1 = q0;
            q1.coeffs() += 0.5f * dt * k1_q.coeffs();
            q1.normalize();
            Eigen::Vector3f omega1 = omega0 + 0.5f * dt * k1_omega;

            // Recalculate forces at midpoint (simplified - using same forces)
            Eigen::Vector3f a1 = a0;
            Eigen::Vector3f alpha1 = alpha0;

            // RK4 stage 2
            Eigen::Vector3f k2_x = v1;
            Eigen::Vector3f k2_v = a1;
            Eigen::Quaternionf omegaQ2(0, omega1.x(), omega1.y(), omega1.z());
            Eigen::Quaternionf k2_q = (omegaQ2 * q1);
            k2_q.coeffs() *= 0.5f;
            Eigen::Vector3f k2_omega = alpha1;

            // RK4 stage 3 (half step)
            Eigen::Vector3f x2 = x0 + 0.5f * dt * k2_x;
            Eigen::Vector3f v2 = v0 + 0.5f * dt * k2_v;
            Eigen::Quaternionf q2 = q0;
            q2.coeffs() += 0.5f * dt * k2_q.coeffs();
            q2.normalize();
            Eigen::Vector3f omega2 = omega0 + 0.5f * dt * k2_omega;

            // Recalculate forces at midpoint (simplified - using same forces)
            Eigen::Vector3f a2 = a0;
            Eigen::Vector3f alpha2 = alpha0;

            // RK4 stage 3
            Eigen::Vector3f k3_x = v2;
            Eigen::Vector3f k3_v = a2;
            Eigen::Quaternionf omegaQ3(0, omega2.x(), omega2.y(), omega2.z());
            Eigen::Quaternionf k3_q = (omegaQ3 * q2);
            k3_q.coeffs() *= 0.5f;
            Eigen::Vector3f k3_omega = alpha2;

            // RK4 stage 4 (full step)
            Eigen::Vector3f x3 = x0 + dt * k3_x;
            Eigen::Vector3f v3 = v0 + dt * k3_v;
            Eigen::Quaternionf q3 = q0;
            q3.coeffs() += dt * k3_q.coeffs();
            q3.normalize();
            Eigen::Vector3f omega3 = omega0 + dt * k3_omega;

            // Recalculate forces at endpoint (simplified - using same forces)
            Eigen::Vector3f a3 = a0;
            Eigen::Vector3f alpha3 = alpha0;

            // RK4 stage 4
            Eigen::Vector3f k4_x = v3;
            Eigen::Vector3f k4_v = a3;
            Eigen::Quaternionf omegaQ4(0, omega3.x(), omega3.y(), omega3.z());
            Eigen::Quaternionf k4_q = (omegaQ4 * q3);
            k4_q.coeffs() *= 0.5f;
            Eigen::Vector3f k4_omega = alpha3;

            // Update state with weighted average
            b->x = x0 + (dt/6.0f) * (k1_x + 2.0f*k2_x + 2.0f*k3_x + k4_x);
            b->xdot = v0 + (dt/6.0f) * (k1_v + 2.0f*k2_v + 2.0f*k3_v + k4_v);

            Eigen::Quaternionf qDot;
            qDot.coeffs() = (1.0f/6.0f) * (k1_q.coeffs() + 2.0f*k2_q.coeffs() + 2.0f*k3_q.coeffs() + k4_q.coeffs());
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();

            b->omega = omega0 + (dt/6.0f) * (k1_omega + 2.0f*k2_omega + 2.0f*k3_omega + k4_omega);
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}

// ——— Implicit Euler Integration ———————————————————————————————————————————
void RigidBodySystem::integrateImplicitEuler(float dt) {
    // This is a semi-implicit implementation with improved stability for marble box
    const float damping            = m_implicitDamping;
    const float gyroscopicDamping  = m_gyroDamping;
    const float maxVelocity        = m_maxLinearVelocity;
    const float maxAngularVelocity = m_maxAngularVelocity;

#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                RigidBody* b = m_bodies[i];
                if (b->color == color && !b->fixed) {
                    // Store initial state for potential rollback
                    Eigen::Vector3f oldXdot = b->xdot;
                    Eigen::Vector3f oldOmega = b->omega;

                    // Compute implicit force components
                    Eigen::Vector3f linearForce = b->f + b->fc;
                    Eigen::Vector3f angularForce = b->tau + b->tauc;

                    // Apply implicit damping to gyroscopic forces for stability
                    Eigen::Vector3f gyroscopicForce = b->omega.cross(b->I * b->omega);
                    gyroscopicForce *= (1.0f - gyroscopicDamping);

                    // Update velocities first (semi-implicit step)
                    b->xdot = damping * b->xdot + dt * (1.0f/b->mass) * linearForce;
                    b->omega = damping * b->omega + dt * b->Iinv * (angularForce - gyroscopicForce);

                    // Apply velocity limiting for stability in marble box scenario
                    float linVelMag = b->xdot.norm();
                    if (linVelMag > maxVelocity) {
                        b->xdot *= (maxVelocity / linVelMag);
                    }

                    float angVelMag = b->omega.norm();
                    if (angVelMag > maxAngularVelocity) {
                        b->omega *= (maxAngularVelocity / angVelMag);
                    }

                    // Update positions with new velocities
                    b->x += dt * b->xdot;

                    // Update orientation using improved quaternion integration
                    Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                    Eigen::Quaternionf qDot = (omegaQ * b->q);
                    qDot.coeffs() *= 0.5f;

                    // Update quaternion with improved stability
                    b->q.coeffs() += dt * qDot.coeffs();
                    b->q.normalize();

                    // Additional stabilization for high angular velocities
                    if (angVelMag > 10.0f) {
                        // Extra normalization for numerical stability
                        b->q.normalize();
                    }

                    // Detect potential numerical instability and correct
                    if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                        !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                        // Restore previous state
                        b->xdot = oldXdot;
                        b->omega = oldOmega;
                        b->x += dt * b->xdot; // Use previous velocity for position update

                        // Normalize quaternion as a safeguard
                        b->q.normalize();
                    }
                } else if (b->fixed) {
                    b->xdot.setZero();
                    b->omega.setZero();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_bodies.size() > 16)
        for (auto b : m_bodies) {
            if (!b->fixed) {
                // Store initial state for potential rollback
                Eigen::Vector3f oldXdot = b->xdot;
                Eigen::Vector3f oldOmega = b->omega;

                // Compute implicit force components
                Eigen::Vector3f linearForce = b->f + b->fc;
                Eigen::Vector3f angularForce = b->tau + b->tauc;

                // Apply implicit damping to gyroscopic forces for stability
                Eigen::Vector3f gyroscopicForce = b->omega.cross(b->I * b->omega);
                gyroscopicForce *= (1.0f - gyroscopicDamping);

                // Update velocities first (semi-implicit step)
                b->xdot = damping * b->xdot + dt * (1.0f/b->mass) * linearForce;
                b->omega = damping * b->omega + dt * b->Iinv * (angularForce - gyroscopicForce);

                // Apply velocity limiting for stability in marble box scenario
                float linVelMag = b->xdot.norm();
                if (linVelMag > maxVelocity) {
                    b->xdot *= (maxVelocity / linVelMag);
                }

                float angVelMag = b->omega.norm();
                if (angVelMag > maxAngularVelocity) {
                    b->omega *= (maxAngularVelocity / angVelMag);
                }

                // Update positions with new velocities
                b->x += dt * b->xdot;

                // Update orientation using improved quaternion integration
                Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                Eigen::Quaternionf qDot = (omegaQ * b->q);
                qDot.coeffs() *= 0.5f;

                // Update quaternion with improved stability
                b->q.coeffs() += dt * qDot.coeffs();
                b->q.normalize();

                // Additional stabilization for high angular velocities
                if (angVelMag > 10.0f) {
                    // Extra normalization for numerical stability
                    b->q.normalize();
                }

                // Detect potential numerical instability and correct
                if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                    !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                    // Restore previous state
                    b->xdot = oldXdot;
                    b->omega = oldOmega;
                    b->x += dt * b->xdot; // Use previous velocity for position update

                    // Normalize quaternion as a safeguard
                    b->q.normalize();
                }
            } else {
                b->xdot.setZero();
                b->omega.setZero();
            }
        }
    }
#else
    for (auto b : m_bodies) {
        if (!b->fixed) {
            // Store initial state for potential rollback
            Eigen::Vector3f oldXdot = b->xdot;
            Eigen::Vector3f oldOmega = b->omega;

            // Compute implicit force components
            Eigen::Vector3f linearForce = b->f + b->fc;
            Eigen::Vector3f angularForce = b->tau + b->tauc;

            // Apply implicit damping to gyroscopic forces for stability
            Eigen::Vector3f gyroscopicForce = b->omega.cross(b->I * b->omega);
            gyroscopicForce *= (1.0f - gyroscopicDamping);

            // Update velocities first (semi-implicit step)
            b->xdot = damping * b->xdot + dt * (1.0f/b->mass) * linearForce;
            b->omega = damping * b->omega + dt * b->Iinv * (angularForce - gyroscopicForce);

            // Apply velocity limiting for stability in marble box scenario
            float linVelMag = b->xdot.norm();
            if (linVelMag > maxVelocity) {
                b->xdot *= (maxVelocity / linVelMag);
            }

            float angVelMag = b->omega.norm();
            if (angVelMag > maxAngularVelocity) {
                b->omega *= (maxAngularVelocity / angVelMag);
            }

            // Update positions with new velocities
            b->x += dt * b->xdot;

            // Update orientation using improved quaternion integration
            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = (omegaQ * b->q);
            qDot.coeffs() *= 0.5f;

            // Update quaternion with improved stability
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();

            // Additional stabilization for high angular velocities
            if (angVelMag > 10.0f) {
                // Extra normalization for numerical stability
                b->q.normalize();
            }

            // Detect potential numerical instability and correct
            if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                // Restore previous state
                b->xdot = oldXdot;
                b->omega = oldOmega;
                b->x += dt * b->xdot; // Use previous velocity for position update

                // Normalize quaternion as a safeguard
                b->q.normalize();
            }
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}

void RigidBodySystem::clear() {
    if (m_resetFunc) m_resetFunc();
    m_collisionDetect->clear();
    for (auto j : m_joints) delete j;
    m_joints.clear();
    for (auto b : m_bodies) delete b;
    m_bodies.clear();
}

void RigidBodySystem::computeInertias() {
#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                if (m_bodies[i]->color == color) {
                    m_bodies[i]->updateInertiaMatrix();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_bodies.size() > 16)
        for (size_t i = 0; i < m_bodies.size(); ++i)
            m_bodies[i]->updateInertiaMatrix();
    }
#else
    for (size_t i = 0; i < m_bodies.size(); ++i)
        m_bodies[i]->updateInertiaMatrix();
#endif
}

const std::vector<Contact*>& RigidBodySystem::getContacts() const {
    return m_collisionDetect->getContacts();
}
std::vector<Contact*>& RigidBodySystem::getContacts() {
    return m_collisionDetect->getContacts();
}

void RigidBodySystem::calcConstraintForces(float dt) {
    int idx = static_cast<int>(m_solverType);
    s_solvers[idx]->setMaxIter(m_solverIter);
    s_solvers[idx]->solve(dt);

#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_joints.empty() && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_joints.size(); ++i) {
                Joint* j = m_joints[i];
                bool processBody0 = j->body0 && j->body0->color == color;
                bool processBody1 = j->body1 && j->body1->color == color;

                Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
                Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;

                if (processBody0) {
                    j->body0->fc   += f0.head<3>();
                    j->body0->tauc += f0.tail<3>();
                }

                if (processBody1) {
                    j->body1->fc   += f1.head<3>();
                    j->body1->tauc += f1.tail<3>();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_joints.size() > 16)
        for (auto j : m_joints) {
            Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
            Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;
            #pragma omp critical
            {
                j->body0->fc   += f0.head<3>();
                j->body0->tauc += f0.tail<3>();
                j->body1->fc   += f1.head<3>();
                j->body1->tauc += f1.tail<3>();
            }
        }
    }
#else
    for (auto j : m_joints) {
        Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
        Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;
        j->body0->fc   += f0.head<3>();
        j->body0->tauc += f0.tail<3>();
        j->body1->fc   += f1.head<3>();
        j->body1->tauc += f1.tail<3>();
    }
#endif

    auto contacts = m_collisionDetect->getContacts();
#ifdef USE_OPENMP
    if (m_useGraphColoring && !contacts.empty() && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < contacts.size(); ++i) {
                Contact* c = contacts[i];
                bool processBody0 = c->body0 && c->body0->color == color && !c->body0->fixed;
                bool processBody1 = c->body1 && c->body1->color == color && !c->body1->fixed;

                Eigen::Vector6f f0 = c->J0.transpose() * c->lambda / dt;
                Eigen::Vector6f f1 = c->J1.transpose() * c->lambda / dt;

                if (processBody0) {
                    c->body0->fc   += f0.head<3>();
                    c->body0->tauc += f0.tail<3>();
                }

                if (processBody1) {
                    c->body1->fc   += f1.head<3>();
                    c->body1->tauc += f1.tail<3>();
                }
            }
        }
    } else {
        #pragma omp parallel for if(contacts.size() > 16)
        for (auto c : contacts) {
            Eigen::Vector6f f0 = c->J0.transpose() * c->lambda / dt;
            Eigen::Vector6f f1 = c->J1.transpose() * c->lambda / dt;
            if (!c->body0->fixed) {
                #pragma omp critical
                {
                    c->body0->fc   += f0.head<3>();
                    c->body0->tauc += f0.tail<3>();
                }
            }
            if (!c->body1->fixed) {
                #pragma omp critical
                {
                    c->body1->fc   += f1.head<3>();
                    c->body1->tauc += f1.tail<3>();
                }
            }
        }
    }
#else
    for (auto c : contacts) {
        Eigen::Vector6f f0 = c->J0.transpose() * c->lambda / dt;
        Eigen::Vector6f f1 = c->J1.transpose() * c->lambda / dt;
        if (!c->body0->fixed) {
            c->body0->fc   += f0.head<3>();
            c->body0->tauc += f0.tail<3>();
        }
        if (!c->body1->fixed) {
            c->body1->fc   += f1.head<3>();
            c->body1->tauc += f1.tail<3>();
        }
    }
#endif
}

// ——— Newton Integration ——————————————————————————————————————————————————
void RigidBodySystem::integrateNewton(float dt) {
    // Newton parameters
    const int maxNewtonIter = 5;               // Maximum Newton iterations
    const float newtonTolerance = 1e-6f;       // Convergence tolerance
    const float dampingFactor = 0.98f;         // Implicit damping factor
    const float maxLinearVel = m_maxLinearVelocity;
    const float maxAngularVel = m_maxAngularVelocity;

#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                RigidBody* b = m_bodies[i];
                if (b->color == color && !b->fixed) {
                    // Store initial state
                    Eigen::Vector3f x0 = b->x;
                    Eigen::Vector3f v0 = b->xdot;
                    Eigen::Quaternionf q0 = b->q;
                    Eigen::Vector3f omega0 = b->omega;

                    // Initial guess is explicit Euler step
                    Eigen::Vector3f force = b->f + b->fc;
                    Eigen::Vector3f torque = b->tau + b->tauc - b->omega.cross(b->I * b->omega);

                    Eigen::Vector3f v_new = v0 + dt * (1.0f/b->mass) * force;
                    Eigen::Vector3f omega_new = omega0 + dt * b->Iinv * torque;

                    // Newton iteration
                    for (int iter = 0; iter < maxNewtonIter; ++iter) {
                        // Compute residual - for simple system, residual is velocity difference
                        Eigen::Vector3f res_v = v_new - (v0 + dt * (1.0f/b->mass) * force);
                        Eigen::Vector3f res_omega = omega_new - (omega0 + dt * b->Iinv * torque);

                        // Check convergence
                        if (res_v.norm() < newtonTolerance && res_omega.norm() < newtonTolerance) {
                            break;
                        }

                        // Jacobian approximation using damped identity
                        // In a true Newton solver, we would have a more accurate Jacobian
                        v_new -= res_v;
                        omega_new -= res_omega;
                    }

                    // Apply velocity limiting for stability
                    float linVelMag = v_new.norm();
                    if (m_limitVelocities && linVelMag > maxLinearVel) {
                        v_new *= (maxLinearVel / linVelMag);
                    }

                    float angVelMag = omega_new.norm();
                    if (m_limitVelocities && angVelMag > maxAngularVel) {
                        omega_new *= (maxAngularVel / angVelMag);
                    }

                    // Apply damping
                    v_new *= dampingFactor;
                    omega_new *= dampingFactor;

                    // Update state
                    b->xdot = v_new;
                    b->omega = omega_new;

                    // Update position
                    b->x = x0 + dt * b->xdot;

                    // Update orientation
                    Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                    Eigen::Quaternionf qDot = omegaQ * q0;
                    qDot.coeffs() *= 0.5f;
                    b->q.coeffs() = q0.coeffs() + dt * qDot.coeffs();
                    b->q.normalize();

                    // Error detection
                    if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                        !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                        // Restore previous state
                        b->x = x0;
                        b->xdot = v0;
                        b->q = q0;
                        b->omega = omega0;
                    }
                } else if (b->fixed) {
                    b->xdot.setZero();
                    b->omega.setZero();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_bodies.size() > 16)
        for (auto b : m_bodies) {
            if (!b->fixed) {
                // Store initial state
                Eigen::Vector3f x0 = b->x;
                Eigen::Vector3f v0 = b->xdot;
                Eigen::Quaternionf q0 = b->q;
                Eigen::Vector3f omega0 = b->omega;

                // Initial guess is explicit Euler step
                Eigen::Vector3f force = b->f + b->fc;
                Eigen::Vector3f torque = b->tau + b->tauc - b->omega.cross(b->I * b->omega);

                Eigen::Vector3f v_new = v0 + dt * (1.0f/b->mass) * force;
                Eigen::Vector3f omega_new = omega0 + dt * b->Iinv * torque;

                // Newton iteration
                for (int iter = 0; iter < maxNewtonIter; ++iter) {
                    // Compute residual - for simple system, residual is velocity difference
                    Eigen::Vector3f res_v = v_new - (v0 + dt * (1.0f/b->mass) * force);
                    Eigen::Vector3f res_omega = omega_new - (omega0 + dt * b->Iinv * torque);

                    // Check convergence
                    if (res_v.norm() < newtonTolerance && res_omega.norm() < newtonTolerance) {
                        break;
                    }

                    // Jacobian approximation using damped identity
                    // In a true Newton solver, we would have a more accurate Jacobian
                    v_new -= res_v;
                    omega_new -= res_omega;
                }

                // Apply velocity limiting for stability
                float linVelMag = v_new.norm();
                if (m_limitVelocities && linVelMag > maxLinearVel) {
                    v_new *= (maxLinearVel / linVelMag);
                }

                float angVelMag = omega_new.norm();
                if (m_limitVelocities && angVelMag > maxAngularVel) {
                    omega_new *= (maxAngularVel / angVelMag);
                }

                // Apply damping
                v_new *= dampingFactor;
                omega_new *= dampingFactor;

                // Update state
                b->xdot = v_new;
                b->omega = omega_new;

                // Update position
                b->x = x0 + dt * b->xdot;

                // Update orientation
                Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                Eigen::Quaternionf qDot = omegaQ * q0;
                qDot.coeffs() *= 0.5f;
                b->q.coeffs() = q0.coeffs() + dt * qDot.coeffs();
                b->q.normalize();

                // Error detection
                if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                    !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                    // Restore previous state
                    b->x = x0;
                    b->xdot = v0;
                    b->q = q0;
                    b->omega = omega0;
                }
            } else {
                b->xdot.setZero();
                b->omega.setZero();
            }
        }
    }
#else
    for (auto b : m_bodies) {
        if (!b->fixed) {
            // Store initial state
            Eigen::Vector3f x0 = b->x;
            Eigen::Vector3f v0 = b->xdot;
            Eigen::Quaternionf q0 = b->q;
            Eigen::Vector3f omega0 = b->omega;

            // Initial guess is explicit Euler step
            Eigen::Vector3f force = b->f + b->fc;
            Eigen::Vector3f torque = b->tau + b->tauc - b->omega.cross(b->I * b->omega);

            Eigen::Vector3f v_new = v0 + dt * (1.0f/b->mass) * force;
            Eigen::Vector3f omega_new = omega0 + dt * b->Iinv * torque;

            // Newton iteration
            for (int iter = 0; iter < maxNewtonIter; ++iter) {
                // Compute residual - for simple system, residual is velocity difference
                Eigen::Vector3f res_v = v_new - (v0 + dt * (1.0f/b->mass) * force);
                Eigen::Vector3f res_omega = omega_new - (omega0 + dt * b->Iinv * torque);

                // Check convergence
                if (res_v.norm() < newtonTolerance && res_omega.norm() < newtonTolerance) {
                    break;
                }

                // Jacobian approximation using damped identity
                // In a true Newton solver, we would have a more accurate Jacobian
                v_new -= res_v;
                omega_new -= res_omega;
            }

            // Apply velocity limiting for stability
            float linVelMag = v_new.norm();
            if (m_limitVelocities && linVelMag > maxLinearVel) {
                v_new *= (maxLinearVel / linVelMag);
            }

            float angVelMag = omega_new.norm();
            if (m_limitVelocities && angVelMag > maxAngularVel) {
                omega_new *= (maxAngularVel / angVelMag);
            }

            // Apply damping
            v_new *= dampingFactor;
            omega_new *= dampingFactor;

            // Update state
            b->xdot = v_new;
            b->omega = omega_new;

            // Update position
            b->x = x0 + dt * b->xdot;

            // Update orientation
            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * q0;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() = q0.coeffs() + dt * qDot.coeffs();
            b->q.normalize();

            // Error detection
            if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                // Restore previous state
                b->x = x0;
                b->xdot = v0;
                b->q = q0;
                b->omega = omega0;
            }
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}