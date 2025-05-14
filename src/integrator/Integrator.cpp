#include "integrator/Integrator.h"
#include "integrator/ExplicitEuler.h"
#include "integrator/ImplicitEuler.h"
#include "integrator/Verlet.h"
#include "integrator/RK4.h"
#include "integrator/SymplecticEuler.h"
#include "integrator/Newton.h"

std::unique_ptr<Integrator> createIntegrator(IntegrationMethod method) {
    switch (method) {
        case IntegrationMethod::EXPLICIT_EULER:
            return std::make_unique<ExplicitEuler>();
        case IntegrationMethod::SYMPLECTIC_EULER:
            return std::make_unique<SymplecticEuler>();
        case IntegrationMethod::VERLET:
            return std::make_unique<Verlet>();
        case IntegrationMethod::RK4:
            return std::make_unique<RK4>();
        case IntegrationMethod::IMPLICIT_EULER:
            return std::make_unique<ImplicitEuler>();
        case IntegrationMethod::NEWTON:
            return std::make_unique<Newton>();
        default:
            return std::make_unique<ExplicitEuler>();
    }
}