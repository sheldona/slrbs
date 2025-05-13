#pragma once
#include "Integrator.h"

class RK4 : public Integrator {
public:
    void integrate(RigidBodySystem& system, float dt) override;
};