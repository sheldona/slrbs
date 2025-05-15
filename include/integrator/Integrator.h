#pragma once
#include "integrator/IntegrationMethod.h"
#include <memory>

class RigidBodySystem;

/// Base class for all integrators.
class Integrator {
public:
    virtual ~Integrator() = default;

    /// Advance the system state by dt using this scheme.
    virtual void integrate(RigidBodySystem& system, float dt) = 0;

    /// Enable/disable OpenMP parallelization
    void setUseOpenMP(bool enable) { m_useOpenMP = enable; }

    /// Check if OpenMP parallelization is enabled
    bool getUseOpenMP() const { return m_useOpenMP; }

protected:
    bool m_useOpenMP = true;
};

/// Factory: create the concrete Integrator for the given method.
std::unique_ptr<Integrator> createIntegrator(IntegrationMethod method, bool useOpenMP = true);