#include "integrator/ExplicitEuler.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_OPENMP
#include <omp.h>
#endif

void ExplicitEuler::integrate(RigidBodySystem& sys, float dt) {
    auto& bodies = sys.getBodies();
    bool useColor = sys.getUseGraphColoring();
    bool useOpenMP = m_useOpenMP;

#ifdef USE_OPENMP
    if (useColor && !bodies.empty()) {
        int numColors = bodies[0]->numColors;
        // Process each color group sequentially
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(useOpenMP)
            for (size_t i = 0; i < bodies.size(); ++i) {
                RigidBody* b = bodies[i];
                if (b->color == color && !b->fixed) {
                    // Update linear velocity
                    b->xdot += dt * (1.0f/b->mass) * (b->f + b->fc);

                    // Update angular velocity (with gyroscopic term)
                    b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                    // Update position
                    b->x += dt * b->xdot;

                    // Update orientation quaternion
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
        // Parallel implementation without graph coloring
        #pragma omp parallel for if(useOpenMP && bodies.size() > 16)
        for (size_t i = 0; i < bodies.size(); ++i) {
            RigidBody* b = bodies[i];
            if (!b->fixed) {
                // Update linear velocity
                b->xdot += dt * (1.0f/b->mass) * (b->f + b->fc);

                // Update angular velocity (with gyroscopic term)
                b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

                // Update position
                b->x += dt * b->xdot;

                // Update orientation quaternion
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
            // Update linear velocity
            b->xdot += dt * (1.0f/b->mass) * (b->f + b->fc);

            // Update angular velocity (with gyroscopic term)
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            // Update position
            b->x += dt * b->xdot;

            // Update orientation quaternion
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