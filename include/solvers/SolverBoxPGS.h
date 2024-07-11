#pragma once

#include "solvers/Solver.h"

// Projected Gauss-Seidel (PGS) Boxed constraint solver.
// This implementation uses a matrix-free approach, where only
// the non-zero blocks of the lead matrix are assembled and stored.
//
class SolverBoxPGS : public Solver
{
public:

    SolverBoxPGS(RigidBodySystem* _rigidBodySystem);

    // Implement PGS method that solves for the constraint impulses in @a m_rigidBodySystem.
    // 
    virtual void solve(float h, int substeps) override;

};
