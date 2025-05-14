#pragma once

#include "integrator/Integrator.h"
#include "rigidbody/RigidBodySystem.h"

/// Concrete Integrator: Explicit Euler scheme (serial or OpenMP-enabled)
class ExplicitEuler : public Integrator {
public:
    /// Advance the system state by dt using explicit Euler
    void integrate(RigidBodySystem& system, float dt) override;
};