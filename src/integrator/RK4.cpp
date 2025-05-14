#include "integrator/RK4.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_OPENMP
#include <omp.h>
#endif

void RK4::integrate(RigidBodySystem& sys, float dt) {
    auto& bodies = sys.getBodies();
    bool useColor = sys.getUseGraphColoring();

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
                    b->q.coeffs() = q0.coeffs() + dt * qDot.coeffs();
                    b->q.normalize();

                    b->omega = omega0 + (dt/6.0f) * (k1_omega + 2.0f*k2_omega + 2.0f*k3_omega + k4_omega);

                    // Check for numerical issues
                    if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                        !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                        // Fallback to simple explicit Euler
                        b->x = x0 + dt * v0;
                        b->xdot = v0 + dt * a0;
                        b->q = q0;
                        b->q.coeffs() += dt * (omegaQ1 * q0).coeffs() * 0.5f;
                        b->q.normalize();
                        b->omega = omega0 + dt * alpha0;
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
                b->q.coeffs() = q0.coeffs() + dt * qDot.coeffs();
                b->q.normalize();

                b->omega = omega0 + (dt/6.0f) * (k1_omega + 2.0f*k2_omega + 2.0f*k3_omega + k4_omega);

                // Check for numerical issues
                if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                    !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                    // Fallback to simple explicit Euler
                    b->x = x0 + dt * v0;
                    b->xdot = v0 + dt * a0;
                    b->q = q0;
                    b->q.coeffs() += dt * (omegaQ1 * q0).coeffs() * 0.5f;
                    b->q.normalize();
                    b->omega = omega0 + dt * alpha0;
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
            b->q.coeffs() = q0.coeffs() + dt * qDot.coeffs();
            b->q.normalize();

            b->omega = omega0 + (dt/6.0f) * (k1_omega + 2.0f*k2_omega + 2.0f*k3_omega + k4_omega);

            // Check for numerical issues
            if (!std::isfinite(b->x.norm()) || !std::isfinite(b->xdot.norm()) ||
                !std::isfinite(b->omega.norm()) || !std::isfinite(b->q.norm())) {
                // Fallback to simple explicit Euler
                b->x = x0 + dt * v0;
                b->xdot = v0 + dt * a0;
                b->q = q0;
                b->q.coeffs() += dt * (omegaQ1 * q0).coeffs() * 0.5f;
                b->q.normalize();
                b->omega = omega0 + dt * alpha0;
            }
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
#endif
}