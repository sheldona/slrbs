#pragma once
#include "Integrator.h"

class ExplicitEuler : public Integrator {
public:
    void integrate(RigidBodySystem& system, float dt) override;
};