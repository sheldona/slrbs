#pragma once

#include "solvers/Solver.h"

// Conjugate gradient solver for constraint systems
//
class SolverConjGradient : public Solver
{
public:
    SolverConjGradient(RigidBodySystem* _rigidBodySystem);

    // Implement Conjugate Gradient method that solves for the constraint impulses
    virtual void solve(float h) override;

    // Getter and setter for convergence tolerance
    float getTolerance() const { return m_tolerance; }
    void setTolerance(float tol) {
        m_tolerance = (tol > 0.0f) ? tol : 1e-8f;
    }

    // Getter and setter for restart interval to improve numerical stability
    int getRestartInterval() const { return m_restartInterval; }
    void setRestartInterval(int interval) {
        m_restartInterval = (interval > 0) ? interval : 10;
    }

private:
    // Convergence tolerance (relative to initial residual)
    float m_tolerance = 1e-8f;

    // Restart the CG algorithm periodically to maintain orthogonality
    int m_restartInterval = 10;
};