#pragma once

#include "solvers/Solver.h"

// Conjugate residual solver
//
class SolverConjResidual : public Solver
{
public:

    SolverConjResidual(RigidBodySystem* _rigidBodySystem);

    // Implement CR method that solves for the constraint impulses in @a m_rigidBodySystem.
    // 
    virtual void solve(float h, int substeps) override;

};
