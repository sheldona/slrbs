#include "ImplicitEulerIntegrator.h"
#include "RigidBodySystem.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#ifdef USE_OPENMP
#include <omp.h>
#endif

void ImplicitEulerIntegrator::integrate(RigidBodySystem& sys, float dt) {
    // ... implementation for implicit Euler ...
}