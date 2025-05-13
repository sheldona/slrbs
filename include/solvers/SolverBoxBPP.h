#pragma once

#include "solvers/Solver.h"

// Block Principal Pivoting (BPP) Boxed LCP solver.
//
class SolverBoxBPP : public Solver
{
public:
    SolverBoxBPP(RigidBodySystem* _rigidBodySystem);

    // Implement Block Principal Pivoting method.
    //
    virtual void solve(float h) override;

private:
    // Maximum number of iterations for the solver
    unsigned int m_maxIter = 100;
};