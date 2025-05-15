#pragma once

#include "solvers/Solver.h"

// Conjugate residual solver
class SolverConjResidual : public Solver
{
public:
    SolverConjResidual(RigidBodySystem* _rigidBodySystem);

    // Implement CR method that solves for the constraint impulses in @a m_rigidBodySystem.
    virtual void solve(float h) override;

    // Set convergence tolerance
    void setTolerance(float tol) { m_tolerance = tol; }

    // Get convergence tolerance
    float getTolerance() const { return m_tolerance; }

    // Set restart interval
    void setRestartInterval(int interval) { m_restartInterval = interval; }

    // Get restart interval
    int getRestartInterval() const { return m_restartInterval; }

private:
    float m_tolerance = 1e-6f;        // Convergence tolerance
    int m_restartInterval = 10;       // Restart interval for numerical stability
};