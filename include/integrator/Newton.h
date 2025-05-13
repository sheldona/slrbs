#pragma once
#include "Integrator.h"

class Newton : public Integrator {
public:
    void integrate(RigidBodySystem& system, float dt) override;
};