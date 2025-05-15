#include "integrator/Verlet.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_OPENMP
#include <omp.h>
#endif

void Verlet::integrate(RigidBodySystem& sys, float dt) {
    auto& bodies = sys.getBodies();
    bool useColor = sys.getUseGraphColoring();
    bool useOpenMP = m_useOpenMP;

#ifdef USE_OPENMP
    if (useColor && !bodies.empty()) {
        int numColors = bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(useOpenMP)
            for (size_t i = 0; i < bodies.size(); ++i) {
                RigidBody* b = bodies[i];
                if (b->color == color && !b->fixed) {
                    // Store previous position for velocity update
                    Eigen::Vector3f prev_x = b->x;
                    Eigen::Vector3f prev_omega = b->omega;
                    Eigen::Quaternionf prev_q = b->q;

                    // Calculate accelerations
                    Eigen::Vector3f accel = (1.0f/b->mass) * (b->f + b->fc);
                    Eigen::Vector3f alpha = b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                    // Update position using velocity and acceleration (velocity Verlet)
                    b->x += b->xdot * dt + 0.5f * accel * dt * dt;

                    // Half-step velocity update
                    Eigen::Vector3f xdot_half = b->xdot + 0.5f * dt * accel;

                    // Full velocity update (will be completed after force recalculation in next step)
                    b->xdot += dt * accel;

                    // Update orientation using half-step method for better energy conservation
                    Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                    Eigen::Quaternionf qDot = omegaQ * b->q;
                    qDot.coeffs() *= 0.5f;

                    // Update quaternion with improved stability
                    b->q.coeffs() += dt * qDot.coeffs();
                    b->q.normalize();

                    // Update angular velocity
                    b->omega += dt * alpha;

                    // Check for numerical instability
                    if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                        !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                        // Restore to previous state with simple Euler step
                        b->x = prev_x + dt * b->xdot;
                        b->q = prev_q;
                        Eigen::Quaternionf safeQDot = (Eigen::Quaternionf(0, prev_omega.x(), prev_omega.y(), prev_omega.z()) * prev_q);
                        safeQDot.coeffs() *= 0.5f;
                        b->q.coeffs() += dt * safeQDot.coeffs();
                        b->q.normalize();
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
                // Store previous position for velocity update
                Eigen::Vector3f prev_x = b->x;
                Eigen::Vector3f prev_omega = b->omega;
                Eigen::Quaternionf prev_q = b->q;

                // Calculate accelerations
                Eigen::Vector3f accel = (1.0f/b->mass) * (b->f + b->fc);
                Eigen::Vector3f alpha = b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                // Update position using velocity and acceleration (velocity Verlet)
                b->x += b->xdot * dt + 0.5f * accel * dt * dt;

                // Half-step velocity update
                Eigen::Vector3f xdot_half = b->xdot + 0.5f * dt * accel;

                // Full velocity update (will be completed after force recalculation in next step)
                b->xdot += dt * accel;

                // Update orientation using half-step method for better energy conservation
                Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
                Eigen::Quaternionf qDot = omegaQ * b->q;
                qDot.coeffs() *= 0.5f;

                // Update quaternion with improved stability
                b->q.coeffs() += dt * qDot.coeffs();
                b->q.normalize();

                // Update angular velocity
                b->omega += dt * alpha;

                // Check for numerical instability
                if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                    !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                    // Restore to previous state with simple Euler step
                    b->x = prev_x + dt * b->xdot;
                    b->q = prev_q;
                    Eigen::Quaternionf safeQDot = (Eigen::Quaternionf(0, prev_omega.x(), prev_omega.y(), prev_omega.z()) * prev_q);
                    safeQDot.coeffs() *= 0.5f;
                    b->q.coeffs() += dt * safeQDot.coeffs();
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
            // Store previous position for velocity update
            Eigen::Vector3f prev_x = b->x;
            Eigen::Vector3f prev_omega = b->omega;
            Eigen::Quaternionf prev_q = b->q;

            // Calculate accelerations
            Eigen::Vector3f accel = (1.0f/b->mass) * (b->f + b->fc);
            Eigen::Vector3f alpha = b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            // Update position using velocity and acceleration (velocity Verlet)
            b->x += b->xdot * dt + 0.5f * accel * dt * dt;

            // Half-step velocity update
            Eigen::Vector3f xdot_half = b->xdot + 0.5f * dt * accel;

            // Full velocity update (will be completed after force recalculation in next step)
            b->xdot += dt * accel;

            // Update orientation using half-step method for better energy conservation
            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;

            // Update quaternion with improved stability
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();

            // Update angular velocity
            b->omega += dt * alpha;

            // Check for numerical instability
            if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                // Restore to previous state with simple Euler step
                b->x = prev_x + dt * b->xdot;
                b->q = prev_q;
                Eigen::Quaternionf safeQDot = (Eigen::Quaternionf(0, prev_omega.x(), prev_omega.y(), prev_omega.z()) * prev_q);
                safeQDot.coeffs() *= 0.5f;
                b->q.coeffs() += dt * safeQDot.coeffs();
                b->q.normalize();
            }
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}