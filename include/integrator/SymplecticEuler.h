#pragma once
#include "Integrator.h"

class SymplecticEuler : public Integrator {
public:
    void integrate(RigidBodySystem& system, float dt) override;
};