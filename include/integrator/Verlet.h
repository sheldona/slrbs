#pragma once
#include "Integrator.h"

class Verlet : public Integrator {
public:
    void integrate(RigidBodySystem& system, float dt) override;
};