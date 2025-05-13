#pragma once
#include "Integrator.h"

class ImplicitEuler : public Integrator {
public:
    void integrate(RigidBodySystem& system, float dt) override;
};