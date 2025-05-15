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

    // Set stabilization parameter (affects constraint forces)
    void setStabilization(float stabilization) {
        m_stabilization = (stabilization > 0.0f) ? stabilization : 1.0f;
        // Update alpha and beta based on stabilization
        m_alpha = m_stabilization * 2.0f;
        m_beta = m_stabilization * m_stabilization * 2.0f;
    }

    float getStabilization() const { return m_stabilization; }

    // Set pivot tolerance
    void setPivotTolerance(float tol) {
        m_pivotTolerance = (tol > 0.0f) ? tol : 1e-5f;
    }

    float getPivotTolerance() const { return m_pivotTolerance; }

    float getAlpha() const { return m_alpha; }

    float getBeta() const { return m_beta; }

    void setAlpha(float alpha) {
        m_alpha = alpha;
    }

    void setBeta(float beta) {
        m_beta = beta;
    }

private:
    // Stabilization parameters
    float m_stabilization = 250.0f;
    float m_alpha = 500.0f;         // 2.0 * stabilization
    float m_beta = 125000.0f;       // 2.0 * stabilization^2
    float m_pivotTolerance = 1e-5f; // Tolerance for pivoting
};