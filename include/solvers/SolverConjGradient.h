#pragma once

#include "solvers/Solver.h"

// Conjugate residual solver
// 
// WARNING: contact handling not supported
//
class SolverConjGradient : public Solver
{
public:

    SolverConjGradient(RigidBodySystem* _rigidBodySystem);

    // Implement CR method that solves for the constraint impulses in @a m_rigidBodySystem.
    // 
    virtual void solve(float h) override;

};
