#include "integrator/ImplicitEuler.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_OPENMP
#include <omp.h>
#endif

void ImplicitEuler::integrate(RigidBodySystem& sys, float dt) {
    auto& bodies = sys.getBodies();
    bool useColor = sys.getUseGraphColoring();

    // Get parameters from the system
    const float damping = sys.getImplicitDamping();
    const float gyroscopicDamping = sys.getGyroscopicDamping();
    const float maxVelocity = sys.getMaxLinearVelocity();
    const float maxAngularVelocity = sys.getMaxAngularVelocity();
    const bool limitVelocities = sys.getVelocityLimitingEnabled();

#ifdef USE_OPENMP
    if (useColor && !bodies.empty()) {
        int numColors = bodies[0]->numColors;

        // Process each color group in sequence for synchronization
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(useOpenMP)
            for (size_t i = 0; i < bodies.size(); ++i) {
                RigidBody* b = bodies[i];
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

                    // Apply velocity limiting for stability
                    if (limitVelocities) {
                        float linVelMag = b->xdot.norm();
                        if (linVelMag > maxVelocity) {
                            b->xdot *= (maxVelocity / linVelMag);
                        }

                        float angVelMag = b->omega.norm();
                        if (angVelMag > maxAngularVelocity) {
                            b->omega *= (maxAngularVelocity / angVelMag);
                        }
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
                    float angVelMag = b->omega.norm();
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
        #pragma omp parallel for if(bodies.size() > 16)
        for (size_t i = 0; i < bodies.size(); ++i) {
            RigidBody* b = bodies[i];
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

                // Apply velocity limiting for stability
                if (limitVelocities) {
                    float linVelMag = b->xdot.norm();
                    if (linVelMag > maxVelocity) {
                        b->xdot *= (maxVelocity / linVelMag);
                    }

                    float angVelMag = b->omega.norm();
                    if (angVelMag > maxAngularVelocity) {
                        b->omega *= (maxAngularVelocity / angVelMag);
                    }
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
                float angVelMag = b->omega.norm();
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
    // Serial implementation (no OpenMP)
    for (auto* b : bodies) {
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

            // Apply velocity limiting for stability
            if (limitVelocities) {
                float linVelMag = b->xdot.norm();
                if (linVelMag > maxVelocity) {
                    b->xdot *= (maxVelocity / linVelMag);
                }

                float angVelMag = b->omega.norm();
                if (angVelMag > maxAngularVelocity) {
                    b->omega *= (maxAngularVelocity / angVelMag);
                }
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
            float angVelMag = b->omega.norm();
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