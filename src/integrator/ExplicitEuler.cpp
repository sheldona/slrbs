#include "ExplicitEuler.h"
#include "RigidBodySystem.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#ifdef USE_OPENMP
#include <omp.h>
#endif

void ExplicitEuler::integrate(RigidBodySystem& sys, float dt) {
    auto& bodies = sys.getBodies();
    bool useColor = sys.getUseGraphColoring();..
}