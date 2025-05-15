#include "solvers/SolverPGSSM.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"
#include "joint/Joint.h"
#include <Eigen/Dense>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace
{
    // Apply matrix-vector operations efficiently (matrix-free approach)
    static inline void applyMatrixVectorOp(const JBlock& G, const Eigen::Vector3f& x, 
                                          const Eigen::Vector3f& y, const float a, 
                                          Eigen::VectorXf& b)
    {
        // No SIMD for MSVC
        for (int i = 0; i < G.rows(); ++i)
        {
            // Linear components (first 3 columns)
            float dot_linear = G(i, 0) * x(0) + G(i, 1) * x(1) + G(i, 2) * x(2);
            
            // Angular components (last 3 columns)
            float dot_angular = G(i, 3) * y(0) + G(i, 4) * y(1) + G(i, 5) * y(2);
            
            // Combined result
            b(i) -= a * (dot_linear + dot_angular);
        }
    }

    // Build right-hand side vector for contacts
    static inline void buildRHS(Contact* c, float h, Eigen::Vector3f& b, float gamma)
    {
        const float hinv = 1.0f / h;
        
        // Initialize RHS: b = -gamma*phi/h
        b = -hinv * gamma * c->phi;

        // Convert to VectorXf for operations
        Eigen::VectorXf tempB(3);
        tempB << b(0), b(1), b(2);

        // Add velocity terms: -J*vel
        if (!c->body0->fixed)
        {
            applyMatrixVectorOp(c->J0, c->body0->xdot, c->body0->omega, 1.0f, tempB);
        }
        if (!c->body1->fixed)
        {
            applyMatrixVectorOp(c->J1, c->body1->xdot, c->body1->omega, 1.0f, tempB);
        }

        // Add force terms: -dt*JMinv*force
        if (!c->body0->fixed)
        {
            applyMatrixVectorOp(c->J0Minv, c->body0->f, c->body0->tau, h, tempB);
        }
        if (!c->body1->fixed)
        {
            applyMatrixVectorOp(c->J1Minv, c->body1->f, c->body1->tau, h, tempB);
        }

        // Convert back to Vector3f
        b << tempB(0), tempB(1), tempB(2);
    }

    // Build right-hand side vector for joints
    static inline void buildRHS(Joint* j, float h, Eigen::VectorXf& b, float gamma)
    {
        const float hinv = 1.0f / h;
        const int dim = j->lambda.rows();
        
        // Initialize RHS: b = -gamma*phi/h
        b = -hinv * gamma * j->phi;

        // Add velocity terms: -J*vel
        if (!j->body0->fixed)
        {
            applyMatrixVectorOp(j->J0, j->body0->xdot, j->body0->omega, 1.0f, b);
        }
        if (!j->body1->fixed)
        {
            applyMatrixVectorOp(j->J1, j->body1->xdot, j->body1->omega, 1.0f, b);
        }

        // Add force terms: -dt*JMinv*force
        if (!j->body0->fixed)
        {
            applyMatrixVectorOp(j->J0Minv, j->body0->f, j->body0->tau, h, b);
        }
        if (!j->body1->fixed)
        {
            applyMatrixVectorOp(j->J1Minv, j->body1->f, j->body1->tau, h, b);
        }
    }

    // Accumulate coupled constraints for contacts
    static inline void accumulateCoupledConstraints(Contact* c, const JBlock& JMinv, 
                                                  RigidBody* body, Eigen::Vector3f& b)
    {
        if (body->fixed)
            return;

        // Convert to VectorXf for operations
        Eigen::VectorXf tempB(3);
        tempB << b(0), b(1), b(2);

        // Accumulate effects from other contacts
        for (Contact* cc : body->contacts)
        {
            if (cc != c)
            {
                if (body == cc->body0)
                    tempB -= JMinv * (cc->J0.transpose() * cc->lambda);
                else
                    tempB -= JMinv * (cc->J1.transpose() * cc->lambda);
            }
        }

        // Accumulate effects from joints
        for (Joint* j : body->joints)
        {
            if (body == j->body0)
                tempB -= JMinv * (j->J0.transpose() * j->lambda);
            else
                tempB -= JMinv * (j->J1.transpose() * j->lambda);
        }

        // Convert back to Vector3f
        b << tempB(0), tempB(1), tempB(2);
    }

    // Accumulate coupled constraints for joints
    static inline void accumulateCoupledConstraints(Joint* j, const JBlock& JMinv, 
                                                  RigidBody* body, Eigen::VectorXf& b)
    {
        if (body->fixed)
            return;

        // Accumulate effects from contacts
        for (Contact* c : body->contacts)
        {
            if (body == c->body0)
                b -= JMinv * (c->J0.transpose() * c->lambda);
            else
                b -= JMinv * (c->J1.transpose() * c->lambda);
        }

        // Accumulate effects from other joints
        for (Joint* jj : body->joints)
        {
            if (jj != j)
            {
                if (body == jj->body0)
                    b -= JMinv * (jj->J0.transpose() * jj->lambda);
                else
                    b -= JMinv * (jj->J1.transpose() * jj->lambda);
            }
        }
    }

    // Box-constrained solver for contacts with friction
    static inline void solveContact(const Eigen::Matrix3f& A, const Eigen::Vector3f& b, 
                                  Eigen::Vector3f& x, const float mu)
    {
        // Normal component (non-penetration constraint)
        x(0) = (b(0) - A(0, 1) * x(1) - A(0, 2) * x(2)) / (A(0, 0) + 1e-6f);
        
        // Project to non-negative values
        if (x(0) < 0.0f) x(0) = 0.0f;

        // Friction components (tangential constraints)
        const float limit = mu * x(0);
        
        // First friction direction
        x(1) = (b(1) - A(1, 0) * x(0) - A(1, 2) * x(2)) / (A(1, 1) + 1e-6f);
        if (x(1) < -limit) x(1) = -limit;
        else if (x(1) > limit) x(1) = limit;

        // Second friction direction
        x(2) = (b(2) - A(2, 0) * x(0) - A(2, 1) * x(1)) / (A(2, 2) + 1e-6f);
        if (x(2) < -limit) x(2) = -limit;
        else if (x(2) > limit) x(2) = limit;
    }

    // Solver for joint constraints (bilateral)
    static inline void solveJoint(const Eigen::MatrixXf& A, const Eigen::VectorXf& b, 
                                Eigen::VectorXf& x)
    {
        // Use LDLT for symmetric positive definite systems
        Eigen::LDLT<Eigen::MatrixXf> ldlt(A);
        x = ldlt.solve(b);
    }

    // Build diagonal matrices for contacts
    static void buildContactMatrices(std::vector<Contact*>& contacts, 
                                   std::vector<Eigen::Matrix3f>& A,
                                   bool useOpenMP)
    {
        const int numContacts = contacts.size();
        A.resize(numContacts);

        #ifdef _OPENMP
        #pragma omp parallel for if(useOpenMP)
        #endif
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            Eigen::Matrix3f diag = Eigen::Matrix3f::Zero();

            // Add contributions from both bodies
            if (!c->body0->fixed)
            {
                diag += c->J0Minv * c->J0.transpose();
            }
            if (!c->body1->fixed)
            {
                diag += c->J1Minv * c->J1.transpose();
            }

            // Add small regularization for stability
            diag(0, 0) += 1e-6f;
            diag(1, 1) += 1e-6f;
            diag(2, 2) += 1e-6f;

            A[i] = diag;
        }
    }

    // Build diagonal matrices for joints
    static void buildJointMatrices(std::vector<Joint*>& joints,
                                 std::vector<Eigen::MatrixXf>& A,
                                 bool useOpenMP)
    {
        const int numJoints = joints.size();
        A.resize(numJoints);

        #ifdef _OPENMP
        #pragma omp parallel for if(useOpenMP)
        #endif
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            const int dim = j->lambda.rows();

            // Initialize with small regularization
            Eigen::MatrixXf diag = 1e-6f * Eigen::MatrixXf::Identity(dim, dim);

            // Add contributions from both bodies
            if (!j->body0->fixed)
            {
                diag += j->J0Minv * j->J0.transpose();
            }
            if (!j->body1->fixed)
            {
                diag += j->J1Minv * j->J1.transpose();
            }

            A[i] = diag;
        }
    }
}

SolverPGSSM::SolverPGSSM(RigidBodySystem* _rigidBodySystem)
    : Solver(_rigidBodySystem),
      m_subIter(3),
      m_gamma(0.3f)
{
}

void SolverPGSSM::solveJoints(std::vector<Joint*>& joints, int numJoints)
{
    if (m_useOpenMP)
    {
        #ifdef _OPENMP
        #pragma omp parallel for
        #endif
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            Eigen::VectorXf x = bjoint[i];

            #pragma omp critical
            {
                // Accumulate coupled constraints
                accumulateCoupledConstraints(j, j->J0Minv, j->body0, x);
                accumulateCoupledConstraints(j, j->J1Minv, j->body1, x);

                // Solve for lambda
                solveJoint(Ajoint[i], x, j->lambda);
            }
        }
    }
    else
    {
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            Eigen::VectorXf x = bjoint[i];

            // Accumulate coupled constraints
            accumulateCoupledConstraints(j, j->J0Minv, j->body0, x);
            accumulateCoupledConstraints(j, j->J1Minv, j->body1, x);

            // Solve for lambda
            solveJoint(Ajoint[i], x, j->lambda);
        }
    }
}

void SolverPGSSM::solveContacts(std::vector<Contact*>& contacts, int numContacts)
{
    if (m_useOpenMP)
    {
        #ifdef _OPENMP
        #pragma omp parallel for
        #endif
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            Eigen::Vector3f x = bcontact[i];

            #pragma omp critical
            {
                // Accumulate coupled constraints
                accumulateCoupledConstraints(c, c->J0Minv, c->body0, x);
                accumulateCoupledConstraints(c, c->J1Minv, c->body1, x);

                // Create a temporary Vector3f, solve, then copy back
                Eigen::Vector3f tempLambda = c->lambda.head<3>();
                solveContact(Acontact[i], x, tempLambda, Contact::mu);
                c->lambda = tempLambda;
            }
        }
    }
    else
    {
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            Eigen::Vector3f x = bcontact[i];

            // Accumulate coupled constraints
            accumulateCoupledConstraints(c, c->J0Minv, c->body0, x);
            accumulateCoupledConstraints(c, c->J1Minv, c->body1, x);

            // Create a temporary Vector3f, solve, then copy back
            Eigen::Vector3f tempLambda = c->lambda.head<3>();
            solveContact(Acontact[i], x, tempLambda, Contact::mu);
            c->lambda = tempLambda;
        }
    }
}

void SolverPGSSM::solveActiveJoints(std::vector<Joint*>& joints, const std::vector<int>& activeIndices)
{
    #ifdef _OPENMP
    #pragma omp parallel for if(m_useOpenMP)
    #endif
    for (int idx = 0; idx < static_cast<int>(activeIndices.size()); ++idx)
    {
        int i = activeIndices[idx];
        if (i >= static_cast<int>(joints.size()))
            continue;

        Joint* j = joints[i];
        Eigen::VectorXf x = bjoint[i];

        #ifdef _OPENMP
        #pragma omp critical
        #endif
        {
            // Accumulate coupled constraints
            accumulateCoupledConstraints(j, j->J0Minv, j->body0, x);
            accumulateCoupledConstraints(j, j->J1Minv, j->body1, x);

            // Solve for lambda
            solveJoint(Ajoint[i], x, j->lambda);
        }
    }
}

void SolverPGSSM::solveActiveContacts(std::vector<Contact*>& contacts, const std::vector<int>& activeIndices)
{
    #ifdef _OPENMP
    #pragma omp parallel for if(m_useOpenMP)
    #endif
    for (int idx = 0; idx < static_cast<int>(activeIndices.size()); ++idx)
    {
        int i = activeIndices[idx];
        if (i >= static_cast<int>(contacts.size()))
            continue;

        Contact* c = contacts[i];
        Eigen::Vector3f x = bcontact[i];

        #ifdef _OPENMP
        #pragma omp critical
        #endif
        {
            // Accumulate coupled constraints
            accumulateCoupledConstraints(c, c->J0Minv, c->body0, x);
            accumulateCoupledConstraints(c, c->J1Minv, c->body1, x);

            // Create a temporary Vector3f, solve, then copy back
            Eigen::Vector3f tempLambda = c->lambda.head<3>();
            solveContact(Acontact[i], x, tempLambda, Contact::mu);
            c->lambda = tempLambda;
        }
    }
}

void SolverPGSSM::updateIndexSets(std::vector<Contact*>& contacts,
                                std::vector<int>& lowerBound,
                                std::vector<int>& upperBound,
                                std::vector<int>& active)
{
    lowerBound.clear();
    upperBound.clear();
    active.clear();

    const int numContacts = contacts.size();
    const float mu = Contact::mu;
    const float tolerance = 1e-6f;

    if (m_useOpenMP)
    {
        // Use temporary storage for parallel classification
        std::vector<std::vector<int>> lowerBoundTemp, upperBoundTemp, activeTemp;

        #ifdef _OPENMP
        int numThreads = omp_get_max_threads();
        lowerBoundTemp.resize(numThreads);
        upperBoundTemp.resize(numThreads);
        activeTemp.resize(numThreads);

        #pragma omp parallel
        {
            int tid = omp_get_thread_num();

            #pragma omp for
            for (int i = 0; i < numContacts; ++i)
            {
                Contact* c = contacts[i];

                // Check if normal force is at lower bound (0)
                if (c->lambda(0) < tolerance) {
                    lowerBoundTemp[tid].push_back(i);
                }
                // Check if friction forces are at boundary
                else if (std::abs(c->lambda(1)) > mu * c->lambda(0) - tolerance ||
                         std::abs(c->lambda(2)) > mu * c->lambda(0) - tolerance) {
                    upperBoundTemp[tid].push_back(i);
                }
                // Otherwise, this is an active constraint
                else {
                    activeTemp[tid].push_back(i);
                }
            }
        }

        // Merge results from all threads
        for (int t = 0; t < numThreads; ++t) {
            lowerBound.insert(lowerBound.end(), lowerBoundTemp[t].begin(), lowerBoundTemp[t].end());
            upperBound.insert(upperBound.end(), upperBoundTemp[t].begin(), upperBoundTemp[t].end());
            active.insert(active.end(), activeTemp[t].begin(), activeTemp[t].end());
        }
        #endif
    }
    else
    {
        // Serial version
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];

            // Check if normal force is at lower bound (0)
            if (c->lambda(0) < tolerance) {
                lowerBound.push_back(i);
            }
            // Check if friction forces are at boundary
            else if (std::abs(c->lambda(1)) > mu * c->lambda(0) - tolerance ||
                     std::abs(c->lambda(2)) > mu * c->lambda(0) - tolerance) {
                upperBound.push_back(i);
            }
            // Otherwise, this is an active constraint
            else {
                active.push_back(i);
            }
        }
    }

    // For joints, all are considered active
    m_activeJoints.clear();
    for (int i = 0; i < static_cast<int>(Ajoint.size()); ++i) {
        m_activeJoints.push_back(i);
    }
}

void SolverPGSSM::solve(float h)
{
    // Get contacts and joints from the rigid body system
    std::vector<Contact*>& contacts = m_rigidBodySystem->getContacts();
    std::vector<Joint*>& joints = m_rigidBodySystem->getJoints();

    const int numContacts = contacts.size();
    const int numJoints = joints.size();

    // If no constraints, early exit
    if (numContacts == 0 && numJoints == 0)
        return;

    // Initialize matrices and vectors
    Ajoint.resize(numJoints);
    Acontact.resize(numContacts);
    bjoint.resize(numJoints);
    bcontact.resize(numContacts);

    // Build diagonal matrices for joints
    if (numJoints > 0) {
        buildJointMatrices(joints, Ajoint, m_useOpenMP);
    }

    // Build diagonal matrices for contacts
    if (numContacts > 0) {
        buildContactMatrices(contacts, Acontact, m_useOpenMP);
    }

    // Compute RHS vectors for joints
    #ifdef _OPENMP
    #pragma omp parallel for if(m_useOpenMP)
    #endif
    for (int i = 0; i < numJoints; ++i) {
        Joint* j = joints[i];
        const int dim = j->lambda.rows();
        bjoint[i].resize(dim);
        buildRHS(j, h, bjoint[i], m_gamma);
        j->lambda.setZero();
    }

    // Compute RHS vectors for contacts
    #ifdef _OPENMP
    #pragma omp parallel for if(m_useOpenMP)
    #endif
    for (int i = 0; i < numContacts; ++i) {
        Contact* c = contacts[i];
        buildRHS(c, h, bcontact[i], m_gamma);
        c->lambda.setZero();
    }
    
    // Initialize index sets for active set strategy
    m_lowerBound.clear();
    m_upperBound.clear();
    m_activeContacts.clear();
    m_activeJoints.clear();
    
    // Main PGS iterations
    for (int iter = 0; iter < m_maxIter; ++iter)
    {
        // First iteration: solve all constraints
        if (iter == 0) {
            // First solve joints (bilateral constraints)
            solveJoints(joints, numJoints);
            
            // Then solve contacts (unilateral constraints with friction)
            solveContacts(contacts, numContacts);
            
            // Update constraint classification
            updateIndexSets(contacts, m_lowerBound, m_upperBound, m_activeContacts);
        }
        else {
            // In subsequent iterations, focus on active constraints
            // Key optimization of the PGSSM method
            
            // Perform sub-iterations on active constraints
            for (int k = 0; k < m_subIter; ++k) {
                // First joints (all joints are considered active)
                solveActiveJoints(joints, m_activeJoints);
                
                // Then active contacts
                solveActiveContacts(contacts, m_activeContacts);
            }
            
            // Update constraint classification
            updateIndexSets(contacts, m_lowerBound, m_upperBound, m_activeContacts);
        }
    }
}