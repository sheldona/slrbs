#pragma once

#include "RigidBodySystem.h"
#include <memory>

enum class IntegrationMethod;

class Integrator {
public:
    virtual ~Integrator() = default;
    virtual void integrate(RigidBodySystem& system, float dt) = 0;
};

std::unique_ptr<Integrator> createIntegrator(IntegrationMethod method);