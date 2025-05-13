#include "Integrator.h"
#include "ExplicitEulerIntegrator.h"
#include "SymplecticEulerIntegrator.h"
#include "VerletIntegrator.h"
#include "RK4Integrator.h"
#include "ImplicitEulerIntegrator.h"
#include "NewtonIntegrator.h"
#include <memory>

std::unique_ptr<Integrator> createIntegrator(IntegrationMethod method) {
    switch(method) {
        case IntegrationMethod::EXPLICIT_EULER:
            return std::make_unique<ExplicitEulerIntegrator>();
        case IntegrationMethod::SYMPLECTIC_EULER:
            return std::make_unique<SymplecticEulerIntegrator>();
        case IntegrationMethod::VERLET:
            return std::make_unique<VerletIntegrator>();
        case IntegrationMethod::RK4:
            return std::make_unique<RK4Integrator>();
        case IntegrationMethod::IMPLICIT_EULER:
            return std::make_unique<ImplicitEulerIntegrator>();
        case IntegrationMethod::NEWTON:
            return std::make_unique<NewtonIntegrator>();
        default:
            return std::make_unique<ExplicitEulerIntegrator>();
    }
}
