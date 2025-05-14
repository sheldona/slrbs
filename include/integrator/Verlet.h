#pragma once
#include "integrator/Integrator.h"
#include "rigidbody/RigidBodySystem.h"

class Verlet : public Integrator {
public:
    void integrate(RigidBodySystem& system, float dt) override;
};