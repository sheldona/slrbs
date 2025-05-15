#include "integrator/Newton.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_OPENMP
#include <omp.h>
#endif

void Newton::integrate(RigidBodySystem& sys, float dt) {
    auto& bodies = sys.getBodies();
    bool useColor = sys.getUseGraphColoring();
    bool useOpenMP = m_useOpenMP;

    // Newton method parameters
    const int maxNewtonIter = 5;               // Maximum Newton iterations
    const float newtonTolerance = 1e-6f;       // Convergence tolerance
    const float dampingFactor = 0.98f;         // Implicit damping factor

    // Get system parameters
    const float maxLinearVel = sys.getMaxLinearVelocity();
    const float maxAngularVel = sys.getMaxAngularVelocity();
    const bool limitVelocities = sys.getVelocityLimitingEnabled();

#ifdef USE_OPENMP
    if (useColor && !bodies.empty()) {
        int numColors = bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(useOpenMP)
            for (size_t i = 0; i < bodies.size(); ++i) {
                RigidBody* b = bodies[i];
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
                    if (limitVelocities) {
                        float linVelMag = v_new.norm();
                        if (linVelMag > maxLinearVel) {
                            v_new *= (maxLinearVel / linVelMag);
                        }

                        float angVelMag = omega_new.norm();
                        if (angVelMag > maxAngularVel) {
                            omega_new *= (maxAngularVel / angVelMag);
                        }
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
        #pragma omp parallel for if(useOpenMP && bodies.size() > 16)
        for (size_t i = 0; i < bodies.size(); ++i) {
            RigidBody* b = bodies[i];
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
                if (limitVelocities) {
                    float linVelMag = v_new.norm();
                    if (linVelMag > maxLinearVel) {
                        v_new *= (maxLinearVel / linVelMag);
                    }

                    float angVelMag = omega_new.norm();
                    if (angVelMag > maxAngularVel) {
                        omega_new *= (maxAngularVel / angVelMag);
                    }
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
    // Serial implementation (no OpenMP)
    for (auto* b : bodies) {
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
            if (limitVelocities) {
                float linVelMag = v_new.norm();
                if (linVelMag > maxLinearVel) {
                    v_new *= (maxLinearVel / linVelMag);
                }

                float angVelMag = omega_new.norm();
                if (angVelMag > maxAngularVel) {
                    omega_new *= (maxAngularVel / angVelMag);
                }
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