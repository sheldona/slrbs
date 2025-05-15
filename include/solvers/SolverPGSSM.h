#pragma once

#include "solvers/Solver.h"
#include <Eigen/Dense>
#include <vector>

// Forward declarations
class RigidBodySystem;
class Contact;
class Joint;

/**
 * Projected Gauss-Seidel Solver with Subspace Minimization (PGSSM).
 * This solver uses an active set strategy to accelerate convergence by
 * focusing computational effort on active constraints.
 */
class SolverPGSSM : public Solver
{
public:
    /**
     * Constructor.
     * @param _rigidBodySystem The rigid body system to solve.
     */
    SolverPGSSM(RigidBodySystem* _rigidBodySystem);
    
    /**
     * Destructor.
     */
    virtual ~SolverPGSSM() = default;

    /**
     * Solves the constraint system for the given time step.
     * Implements active set strategy for improved convergence.
     * @param h The time step.
     */
    virtual void solve(float h) override;

    /**
     * Set the number of sub-iterations for active constraints
     */
    void setSubIterations(int subIter) { m_subIter = (subIter > 0) ? subIter : 1; }

    /**
     * Get the number of sub-iterations for active constraints
     */
    int getSubIterations() const { return m_subIter; }

    /**
     * Set the Baumgarte stabilization parameter
     */
    void setGamma(float gamma) { m_gamma = (gamma > 0.0f && gamma < 1.0f) ? gamma : 0.3f; }

    /**
     * Get the Baumgarte stabilization parameter
     */
    float getGamma() const { return m_gamma; }

private:
    /**
     * Solves all joint constraints.
     * @param joints The joint constraints.
     * @param numJoints The number of joints.
     */
    void solveJoints(std::vector<Joint*>& joints, int numJoints);

    /**
     * Solves all contact constraints.
     * @param contacts The contact constraints.
     * @param numContacts The number of contacts.
     */
    void solveContacts(std::vector<Contact*>& contacts, int numContacts);

    /**
     * Solves only active joint constraints.
     * @param joints The joint constraints.
     * @param activeIndices Indices of active joints.
     */
    void solveActiveJoints(std::vector<Joint*>& joints, const std::vector<int>& activeIndices);

    /**
     * Solves only active contact constraints.
     * @param contacts The contact constraints.
     * @param activeIndices Indices of active contacts.
     */
    void solveActiveContacts(std::vector<Contact*>& contacts, const std::vector<int>& activeIndices);

    /**
     * Updates the index sets for the active set strategy.
     * @param contacts The contact constraints.
     * @param lowerBound Output: indices of constraints at lower bound.
     * @param upperBound Output: indices of constraints at upper bound.
     * @param active Output: indices of active constraints.
     */
    void updateIndexSets(std::vector<Contact*>& contacts,
                        std::vector<int>& lowerBound,
                        std::vector<int>& upperBound,
                        std::vector<int>& active);

    // Solver parameters
    int m_subIter = 3;       // Number of sub-iterations for active constraints
    float m_gamma = 0.3f;    // Baumgarte stabilization parameter
    
    // Matrices and vectors used by the solver
    std::vector<Eigen::MatrixXf> Ajoint;      // System matrices for joints
    std::vector<Eigen::Matrix3f> Acontact;    // System matrices for contacts
    std::vector<Eigen::VectorXf> bjoint;      // Right-hand side vectors for joints
    std::vector<Eigen::Vector3f> bcontact;    // Right-hand side vectors for contacts
    
    // Index sets for active set strategy
    std::vector<int> m_lowerBound;        // Indices of constraints at lower bound
    std::vector<int> m_upperBound;        // Indices of constraints at upper bound
    std::vector<int> m_activeContacts;    // Indices of active contact constraints
    std::vector<int> m_activeJoints;      // Indices of active joint constraints
};