#pragma once

#include "integrator/Integrator.h"
#include "rigidbody/RigidBodySystem.h"


/// Concrete Integrator: Symplectic Euler scheme (serial or OpenMP-enabled)
class SymplecticEuler : public Integrator {
public:
    /// Advance the system state by dt using symplectic Euler
    void integrate(RigidBodySystem& system, float dt) override;
};