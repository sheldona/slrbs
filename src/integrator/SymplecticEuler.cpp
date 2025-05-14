#include "integrator/SymplecticEuler.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_OPENMP
#include <omp.h>
#endif

void SymplecticEuler::integrate(RigidBodySystem& sys, float dt) {
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
                    // First update velocities
                    b->xdot += dt * (1.0f/b->mass) * (b->f + b->fc);
                    b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                    // Then update positions with the new velocities
                    b->x += dt * b->xdot;

                    // Update orientation with the new angular velocity
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
        #pragma omp parallel for if(useOpenMP && bodies.size() > 16)
        for (size_t i = 0; i < bodies.size(); ++i) {
            RigidBody* b = bodies[i];
            if (!b->fixed) {
                // First update velocities
                b->xdot += dt * (1.0f/b->mass) * (b->f + b->fc);
                b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                // Then update positions with the new velocities
                b->x += dt * b->xdot;

                // Update orientation with the new angular velocity
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
    // Serial implementation (no OpenMP)
    for (auto* b : bodies) {
        if (!b->fixed) {
            // First update velocities
            b->xdot += dt * (1.0f/b->mass) * (b->f + b->fc);
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            // Then update positions with the new velocities
            b->x += dt * b->xdot;

            // Update orientation with the new angular velocity
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