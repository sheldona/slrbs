#include "integrator/Integrator.h"
#include "integrator/ExplicitEuler.h"
#include "integrator/ImplicitEuler.h"
#include "integrator/Verlet.h"
#include "integrator/RK4.h"
#include "integrator/SymplecticEuler.h"
#include "integrator/Newton.h"

std::unique_ptr<Integrator> createIntegrator(IntegrationMethod method, bool useOpenMP) {
    switch (method) {
        case IntegrationMethod::EXPLICIT_EULER: {
            auto integrator = std::make_unique<ExplicitEuler>();
            integrator->setUseOpenMP(useOpenMP);
            return integrator;
        }
        case IntegrationMethod::SYMPLECTIC_EULER: {
            auto integrator = std::make_unique<SymplecticEuler>();
            integrator->setUseOpenMP(useOpenMP);
            return integrator;
        }
        case IntegrationMethod::VERLET: {
            auto integrator = std::make_unique<Verlet>();
            integrator->setUseOpenMP(useOpenMP);
            return integrator;
        }
        case IntegrationMethod::RK4: {
            auto integrator = std::make_unique<RK4>();
            integrator->setUseOpenMP(useOpenMP);
            return integrator;
        }
        case IntegrationMethod::IMPLICIT_EULER: {
            auto integrator = std::make_unique<ImplicitEuler>();
            integrator->setUseOpenMP(useOpenMP);
            return integrator;
        }
        case IntegrationMethod::NEWTON: {
            auto integrator = std::make_unique<Newton>();
            integrator->setUseOpenMP(useOpenMP);
            return integrator;
        }
        default: {
            auto integrator = std::make_unique<ExplicitEuler>();
            integrator->setUseOpenMP(useOpenMP);
            return integrator;
        }
    }
}