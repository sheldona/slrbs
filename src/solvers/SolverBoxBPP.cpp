#include "solvers/SolverBoxBPP.h"

#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"

#include <Eigen/Dense>

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace
{
    enum eIndexSet { kFree = 0, kLower, kUpper, kIgnore };

    static inline void multAndSub(const JBlock& G, const Eigen::Vector3f& x, const Eigen::Vector3f& y, const float a, Eigen::Ref<Eigen::VectorXf> b)
    {
        b -= a * G.col(0) * x(0);
        b -= a * G.col(1) * x(1);
        b -= a * G.col(2) * x(2);
        b -= a * G.col(3) * y(0);
        b -= a * G.col(4) * y(1);
        b -= a * G.col(5) * y(2);
    }

    // Update the box bounds, lower and upper, of the constraint impulses.
    // The value in Contact::lambda is used for updating the bounds.
    static inline void updateBounds(std::vector<Contact*>& contacts, Eigen::VectorXf& lower, Eigen::VectorXf& upper, bool useOpenMP, float stabilization)
    {
#ifdef USE_OPENMP
        #pragma omp parallel for if(useOpenMP && contacts.size() > 16)
#endif
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            Contact* c = contacts[i];
            const unsigned int dim = c->J0.rows();

            // Non-interpenetration row.
            lower(c->idx) = 0.0f;
            upper(c->idx) = std::numeric_limits<float>::max();

            // Friction rows.
            // Compute the box bounds as [-mu*lambda_n, mu*lambda_n]
            // which is an approximation of the isotropic Coulomb friction cone.
            for (unsigned int j = 1; j < dim; ++j)
            {
                lower(c->idx + j) = -c->mu * c->lambda(0);
                upper(c->idx + j) = c->mu * c->lambda(0);
            }
        }
    }

    // Build the rhs vector of the Schur complement linear system.
    static inline void buildRHS(const std::vector<Joint*>& joints, const std::vector<Contact*>& contacts,
                              float h, Eigen::VectorXf& b, bool useOpenMP, float stabilization, float alpha, float beta)
    {
        const float hinv = 1.0f / h;

        // Process joints
#ifdef USE_OPENMP
        if (useOpenMP && joints.size() > 16) {
            #pragma omp parallel
            {
                #pragma omp for
                for (size_t i = 0; i < joints.size(); ++i) {
                    Joint* j = joints[i];
                    const unsigned int dim = j->dim;

                    // Thread-local segment to avoid race conditions
                    Eigen::VectorXf localB = -hinv * j->phi * (h * beta / (h * beta + alpha)) * Eigen::VectorXf::Ones(dim);

                    if (!j->body0->fixed) {
                        multAndSub(j->J0Minv, j->body0->f, j->body0->tau, h, localB);
                        multAndSub(j->J0, j->body0->xdot, j->body0->omega, 1.0f, localB);
                    }
                    if (!j->body1->fixed) {
                        multAndSub(j->J1Minv, j->body1->f, j->body1->tau, h, localB);
                        multAndSub(j->J1, j->body1->xdot, j->body1->omega, 1.0f, localB);
                    }

                    // Critical section to update the shared result vector
                    #pragma omp critical
                    {
                        b.segment(j->idx, dim) = localB;
                    }
                }
            }
        } else
#endif
        {
            // Serial version
            for (auto j : joints)
            {
                const unsigned int dim = j->dim;
                b.segment(j->idx, dim) = -hinv * j->phi * (h * beta / (h * beta + alpha));

                if (!j->body0->fixed)
                {
                    multAndSub(j->J0Minv, j->body0->f, j->body0->tau, h, b.segment(j->idx, dim));
                    multAndSub(j->J0, j->body0->xdot, j->body0->omega, 1.0f, b.segment(j->idx, dim));
                }
                if (!j->body1->fixed)
                {
                    multAndSub(j->J1Minv, j->body1->f, j->body1->tau, h, b.segment(j->idx, dim));
                    multAndSub(j->J1, j->body1->xdot, j->body1->omega, 1.0f, b.segment(j->idx, dim));
                }
            }
        }

        // Process contacts
#ifdef USE_OPENMP
        if (useOpenMP && contacts.size() > 16) {
            #pragma omp parallel
            {
                #pragma omp for
                for (size_t i = 0; i < contacts.size(); ++i) {
                    Contact* c = contacts[i];
                    const unsigned int dim = c->dim;

                    // Thread-local segment to avoid race conditions
                    Eigen::VectorXf localB = -hinv * c->phi * (h * beta / (h * beta + alpha)) * Eigen::VectorXf::Ones(dim);

                    if (!c->body0->fixed) {
                        multAndSub(c->J0Minv, c->body0->f, c->body0->tau, h, localB);
                        multAndSub(c->J0, c->body0->xdot, c->body0->omega, 1.0f, localB);
                    }
                    if (!c->body1->fixed) {
                        multAndSub(c->J1Minv, c->body1->f, c->body1->tau, h, localB);
                        multAndSub(c->J1, c->body1->xdot, c->body1->omega, 1.0f, localB);
                    }

                    // Critical section to update the shared result vector
                    #pragma omp critical
                    {
                        b.segment(c->idx, dim) = localB;
                    }
                }
            }
        } else
#endif
        {
            // Serial version
            for (auto c : contacts)
            {
                const unsigned int dim = c->dim;
                b.segment(c->idx, dim) = -hinv * c->phi * (h * beta / (h * beta + alpha));

                if (!c->body0->fixed)
                {
                    multAndSub(c->J0Minv, c->body0->f, c->body0->tau, h, b.segment(c->idx, dim));
                    multAndSub(c->J0, c->body0->xdot, c->body0->omega, 1.0f, b.segment(c->idx, dim));
                }
                if (!c->body1->fixed)
                {
                    multAndSub(c->J1Minv, c->body1->f, c->body1->tau, h, b.segment(c->idx, dim));
                    multAndSub(c->J1, c->body1->xdot, c->body1->omega, 1.0f, b.segment(c->idx, dim));
                }
            }
        }
    }

    // Build the Schur complement system using the contact constraints.
    static inline void buildMatrix(const std::vector<Joint*>& joints, const std::vector<Contact*>& contacts,
                                 float h, Eigen::MatrixXf& A, bool useOpenMP, float stabilization, float alpha, float beta)
    {
        // Processing is mostly sequential due to matrix dependencies
        // But we could parallelize within joint/contact loops if needed
        for (auto j : joints)
        {
            const unsigned int dim = j->dim;
            const float eps = 1e-5f + 1.0f / (h * h * beta + alpha);

            A.block(j->idx, j->idx, dim, dim) = eps * Eigen::MatrixXf::Identity(dim, dim);

            if (!j->body0->fixed)
            {
                A.block(j->idx, j->idx, dim, dim) += j->J0Minv * j->J0.transpose();
                for (auto cc : j->body0->contacts)
                {
                    const int ddim = cc->dim;
                    if (cc->body0 == j->body0)
                    {
                        A.block(j->idx, cc->idx, dim, ddim) += j->J0Minv * cc->J0.transpose();
                    }
                    else
                    {
                        A.block(j->idx, cc->idx, dim, ddim) += j->J0Minv * cc->J1.transpose();
                    }
                }
                for (auto jj : j->body0->joints)
                {
                    if (jj != j)
                    {
                        const int ddim = jj->dim;
                        if (jj->body0 == j->body0)
                        {
                            A.block(j->idx, jj->idx, dim, ddim) += j->J0Minv * jj->J0.transpose();
                        }
                        else
                        {
                            A.block(j->idx, jj->idx, dim, ddim) += j->J0Minv * jj->J1.transpose();
                        }
                    }
                }
            }

            if (!j->body1->fixed)
            {
                A.block(j->idx, j->idx, dim, dim) += j->J1Minv * j->J1.transpose();

                for (auto cc : j->body1->contacts)
                {
                    const int ddim = cc->dim;
                    if (cc->body0 == j->body1)
                    {
                        A.block(j->idx, cc->idx, dim, ddim) += j->J1Minv * cc->J0.transpose();
                    }
                    else
                    {
                        A.block(j->idx, cc->idx, dim, ddim) += j->J1Minv * cc->J1.transpose();
                    }
                }
                for (auto jj : j->body1->joints)
                {
                    if (jj != j)
                    {
                        const int ddim = j->J1.rows();
                        if (jj->body0 == j->body1)
                        {
                            A.block(j->idx, jj->idx, dim, ddim) += j->J1Minv * jj->J0.transpose();
                        }
                        else
                        {
                            A.block(j->idx, jj->idx, dim, ddim) += j->J1Minv * jj->J1.transpose();
                        }
                    }
                }
            }
        }
        for (auto c : contacts)
        {
            const unsigned int dim = c->dim;
            const float eps = 1.0f / (h * h * beta + alpha);

            A.block(c->idx, c->idx, dim, dim) = 1e-10f * Eigen::MatrixXf::Identity(dim, dim);
            A(c->idx, c->idx) += eps;

            if (!c->body0->fixed)
            {
                A.block(c->idx, c->idx, dim, dim) += c->J0Minv * c->J0.transpose();
                for (auto cc : c->body0->contacts)
                {
                    if (cc != c)
                    {
                        const int ddim = cc->dim;
                        if (cc->body0 == c->body0)
                        {
                            A.block(c->idx, cc->idx, dim, ddim) += c->J0Minv * cc->J0.transpose();
                        }
                        else
                        {
                            A.block(c->idx, cc->idx, dim, ddim) += c->J0Minv * cc->J1.transpose();
                        }
                    }
                }
                for (auto jj : c->body0->joints)
                {
                    const int ddim = jj->dim;
                    if (jj->body0 == c->body0)
                    {
                        A.block(c->idx, jj->idx, dim, ddim) += c->J0Minv * jj->J0.transpose();
                    }
                    else
                    {
                        A.block(c->idx, jj->idx, dim, ddim) += c->J0Minv * jj->J1.transpose();
                    }
                }
            }

            if (!c->body1->fixed)
            {
                A.block(c->idx, c->idx, dim, dim) += c->J1Minv * c->J1.transpose();

                for (auto cc : c->body1->contacts)
                {
                    if (cc != c)
                    {
                        const int ddim = cc->dim;
                        if (cc->body0 == c->body1)
                        {
                            A.block(c->idx, cc->idx, dim, ddim) += c->J1Minv * cc->J0.transpose();
                        }
                        else
                        {
                            A.block(c->idx, cc->idx, dim, ddim) += c->J1Minv * cc->J1.transpose();
                        }
                    }
                }
                for (auto jj : c->body1->joints)
                {
                    const int ddim = jj->dim;
                    if (jj->body0 == c->body1)
                    {
                        A.block(c->idx, jj->idx, dim, ddim) += c->J1Minv * jj->J0.transpose();
                    }
                    else
                    {
                        A.block(c->idx, jj->idx, dim, ddim) += c->J1Minv * jj->J1.transpose();
                    }
                }
            }
        }
    }

    // Pivoting rules.
    // The index set is updates based on the LCP variables x and v.
    // Specifically, 'free' variables are pivoted to the 'tight' set if the lower or upper bounds are violated.
    // Similarly, 'tight' variables are pivoted to the 'free' set if the residual velocity v has the wrong sign.
    static inline unsigned int pivot(Eigen::VectorXi& idx, Eigen::VectorXf& x, const Eigen::VectorXf& l,
                                  const Eigen::VectorXf& u, const Eigen::VectorXf& v, bool useOpenMP)
    {
        static const float tol = 1e-5f;
        unsigned int numPivots = 0;
        const unsigned int n = idx.rows();

#ifdef USE_OPENMP
        if (useOpenMP && n > 500) {
            // Split pivoting into separate atomic counter and parallel loop
            std::atomic<unsigned int> atomicPivots(0);

            #pragma omp parallel
            {
                #pragma omp for
                for (int j = 0; j < static_cast<int>(n); ++j) {
                    // By default, assume all variables belong to the 'free' set.
                    int new_idx = kFree;

                    if (idx[j] == kIgnore) continue;

                    if (idx[j] == kFree && x[j] <= l[j])     // case: free variable, lower bound
                    {
                        new_idx = kLower;
                    }
                    else if (idx[j] == kFree && x[j] >= u[j])  // case: free variable, upper bound
                    {
                        new_idx = kUpper;
                    }
                    else if (idx[j] == kLower && v[j] > -tol)    // case: lower tight variable, +ve velocity
                    {
                        new_idx = kLower;
                    }
                    else if (idx[j] == kUpper && v[j] < tol)     // case: upper tight variable, -ve velocity
                    {
                        new_idx = kUpper;
                    }

                    if (new_idx != idx[j])
                    {
                        atomicPivots++;
                        idx[j] = new_idx;
                    }
                }
            }
            numPivots = atomicPivots;
        } else
#endif
        {
            // Serial version
            for (unsigned int j = 0; j < n; ++j)
            {
                // By default, assume all variables belong to the 'free' set.
                int new_idx = kFree;

                if (idx[j] == kIgnore) continue;

                if (idx[j] == kFree && x[j] <= l[j])     // case: free variable, lower bound
                {
                    new_idx = kLower;
                }
                else if (idx[j] == kFree && x[j] >= u[j])  // case: free variable, upper bound
                {
                    new_idx = kUpper;
                }
                else if (idx[j] == kLower && v[j] > -tol)    // case: lower tight variable, +ve velocity
                {
                    new_idx = kLower;
                }
                else if (idx[j] == kUpper && v[j] < tol)     // case: upper tight variable, -ve velocity
                {
                    new_idx = kUpper;
                }

                if (new_idx != idx[j])
                {
                    ++numPivots;
                    idx[j] = new_idx;
                }
            }
        }
        return numPivots;
    }

    // Returns the indices of tight variables in tightIdx.
    static inline unsigned int tightIndices(const Eigen::VectorXi& idx, std::vector<int>& tightIdx, bool useOpenMP)
    {
        const unsigned int n = idx.rows();
        tightIdx.clear();

#ifdef USE_OPENMP
        if (useOpenMP && n > 500) {
            // First count tight indices in parallel
            std::vector<std::vector<int>> localTightLists;

            #pragma omp parallel
            {
                #pragma omp single
                {
                    localTightLists.resize(omp_get_num_threads());
                }

                int tid = omp_get_thread_num();
                std::vector<int>& localTight = localTightLists[tid];

                #pragma omp for
                for (int i = 0; i < static_cast<int>(n); ++i) {
                    if (idx[i] == kLower || idx[i] == kUpper) {
                        localTight.push_back(i);
                    }
                }
            }

            // Reserve space for the total size
            int totalSize = 0;
            for (const auto& local : localTightLists) {
                totalSize += local.size();
            }
            tightIdx.reserve(totalSize);

            // Merge results
            for (const auto& local : localTightLists) {
                tightIdx.insert(tightIdx.end(), local.begin(), local.end());
            }

            return tightIdx.size();
        } else
#endif
        {
            // Serial version
            unsigned int numTight = 0;
            for (unsigned int i = 0; i < n; ++i)
            {
                if (idx[i] == kLower || idx[i] == kUpper)
                {
                    ++numTight;
                    tightIdx.push_back(i);
                }
            }
            return numTight;
        }
    }

    // Returns the indices of free variables in freeIdx.
    static inline unsigned int freeIndices(const Eigen::VectorXi& idx, std::vector<int>& freeIdx, bool useOpenMP)
    {
        const unsigned int n = idx.rows();
        freeIdx.clear();

#ifdef USE_OPENMP
        if (useOpenMP && n > 500) {
            // First count free indices in parallel
            std::vector<std::vector<int>> localFreeLists;

            #pragma omp parallel
            {
                #pragma omp single
                {
                    localFreeLists.resize(omp_get_num_threads());
                }

                int tid = omp_get_thread_num();
                std::vector<int>& localFree = localFreeLists[tid];

                #pragma omp for
                for (int i = 0; i < static_cast<int>(n); ++i) {
                    if (idx[i] == kFree) {
                        localFree.push_back(i);
                    }
                }
            }

            // Reserve space for the total size
            int totalSize = 0;
            for (const auto& local : localFreeLists) {
                totalSize += local.size();
            }
            freeIdx.reserve(totalSize);

            // Merge results
            for (const auto& local : localFreeLists) {
                freeIdx.insert(freeIdx.end(), local.begin(), local.end());
            }

            return freeIdx.size();
        } else
#endif
        {
            // Serial version
            unsigned int numFree = 0;
            for (unsigned int i = 0; i < n; ++i)
            {
                if (idx[i] == kFree)
                {
                    ++numFree;
                    freeIdx.push_back(i);
                }
            }
            return numFree;
        }
    }

    // Solve the principal sub-problem comprising the free variables:
    //      Aff * xf = bf - Aft * xt
    //
    //  where Aff is the sub-matrix of free variables, bf are the corresponding entries in the rhs vector,
    //  Aft is the sub-matrix that couples the tight variables and the free variables, and
    //  xt are the tight variables whose values is determined by the lower and upper bounds (l and u).
    //
    // Inputs:
    //    A - the lead matrix
    //    b - the rhs vector
    //    idx - the index set of all variables
    //    l - the lower bounds
    //    u - the upper bounds
    //
    // Outputs:
    //    x - the solution of constraint impulses (free and tight variables)
    static inline void solvePrincipalSubproblem(const Eigen::MatrixXf& A,
        const Eigen::VectorXf& b,
        const Eigen::VectorXi& idx,
        const Eigen::VectorXf& l,
        const Eigen::VectorXf& u,
        Eigen::VectorXf& x,
        bool useOpenMP)
    {
        std::vector<int> freeIdx, tightIdx;
        const unsigned int numTight = tightIndices(idx, tightIdx, useOpenMP);
        const unsigned int numFree = freeIndices(idx, freeIdx, useOpenMP);

        if (numFree > 0)
        {
            Eigen::MatrixXf Aff(numFree, numFree);
            Eigen::VectorXf bf(numFree);

            // Build sub-matrix using free indices
#ifdef USE_OPENMP
            if (useOpenMP && numFree > 100) {
                #pragma omp parallel for collapse(2)
                for (int j = 0; j < static_cast<int>(numFree); ++j) {
                    for (int i = 0; i < static_cast<int>(numFree); ++i) {
                        Aff(i, j) = A(freeIdx[i], freeIdx[j]);
                    }
                }
            } else
#endif
            {
                // Serial version
                for (unsigned int j = 0; j < numFree; ++j) {
                    for (unsigned int i = 0; i < numFree; ++i) {
                        Aff(i, j) = A(freeIdx[i], freeIdx[j]);
                    }
                }
            }

            // Build rhs vector using free indices
#ifdef USE_OPENMP
            if (useOpenMP && numFree > 100) {
                #pragma omp parallel for
                for (int i = 0; i < static_cast<int>(numFree); ++i) {
                    bf(i) = b(freeIdx[i]);
                }
            } else
#endif
            {
                // Serial version
                for (unsigned int i = 0; i < numFree; ++i) {
                    bf(i) = b(freeIdx[i]);
                }
            }

            // Update rhs vector with impulses from tight indices
            //  e.g.     bf -= A_ft * x_t
#ifdef USE_OPENMP
            if (useOpenMP && numFree > 100 && numTight > 100) {
                #pragma omp parallel
                {
                    // Use local vectors to avoid race conditions
                    Eigen::VectorXf localBf = Eigen::VectorXf::Zero(numFree);

                    #pragma omp for
                    for (int j = 0; j < static_cast<int>(numTight); ++j) {
                        for (unsigned int i = 0; i < numFree; ++i) {
                            if (idx[tightIdx[j]] == kLower) {
                                localBf(i) -= A(freeIdx[i], tightIdx[j]) * l(tightIdx[j]);
                            }
                            else if (idx[tightIdx[j]] == kUpper) {
                                localBf(i) -= A(freeIdx[i], tightIdx[j]) * u(tightIdx[j]);
                            }
                        }
                    }

                    // Reduce results
                    #pragma omp critical
                    {
                        bf += localBf;
                    }
                }
            } else
#endif
            {
                // Serial version
                for (unsigned int j = 0; j < numTight; ++j) {
                    for (unsigned int i = 0; i < numFree; ++i) {
                        if (idx[tightIdx[j]] == kLower) {
                            bf(i) -= A(freeIdx[i], tightIdx[j]) * l(tightIdx[j]);
                        }
                        else if (idx[tightIdx[j]] == kUpper) {
                            bf(i) -= A(freeIdx[i], tightIdx[j]) * u(tightIdx[j]);
                        }
                    }
                }
            }

            // Cholesky solve for the principal sub-problem.
            Eigen::LDLT<Eigen::MatrixXf> ldlt(Aff);
            const Eigen::VectorXf xf = ldlt.solve(bf);

            // Update free variables with the solution
#ifdef USE_OPENMP
            if (useOpenMP && numFree > 100) {
                #pragma omp parallel for
                for (int i = 0; i < static_cast<int>(numFree); ++i) {
                    x(freeIdx[i]) = xf(i);
                }
            } else
#endif
            {
                // Serial version
                for (unsigned int i = 0; i < numFree; ++i) {
                    x(freeIdx[i]) = xf(i);
                }
            }

            // Update tight variables with values from lower/upper bounds.
#ifdef USE_OPENMP
            if (useOpenMP && numTight > 100) {
                #pragma omp parallel for
                for (int i = 0; i < static_cast<int>(numTight); ++i) {
                    if (idx[tightIdx[i]] == kLower) {
                        x(tightIdx[i]) = l(tightIdx[i]);
                    }
                    else if (idx[tightIdx[i]] == kUpper) {
                        x(tightIdx[i]) = u(tightIdx[i]);
                    }
                }
            } else
#endif
            {
                // Serial version
                for (unsigned int i = 0; i < numTight; ++i) {
                    if (idx[tightIdx[i]] == kLower) {
                        x(tightIdx[i]) = l(tightIdx[i]);
                    }
                    else if (idx[tightIdx[i]] == kUpper) {
                        x(tightIdx[i]) = u(tightIdx[i]);
                    }
                }
            }
        }
    }

    // Distribute impulses to joints and contacts
    inline void updateJointsContacts(const Eigen::VectorXf& x, std::vector<Joint*>& joints,
                                   std::vector<Contact*>& contacts, bool useOpenMP)
    {
        // Distribute impulses to the joints
#ifdef USE_OPENMP
        if (useOpenMP && joints.size() > 16) {
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
                joints[i]->lambda = x.segment(joints[i]->idx, joints[i]->dim);
            }
        } else
#endif
        {
            // Serial version
            for (auto j : joints) {
                j->lambda = x.segment(j->idx, j->dim);
            }
        }

        // Distribute impulses to the contacts
#ifdef USE_OPENMP
        if (useOpenMP && contacts.size() > 16) {
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
                contacts[i]->lambda = x.segment(contacts[i]->idx, contacts[i]->dim);
            }
        } else
#endif
        {
            // Serial version
            for (auto c : contacts) {
                c->lambda = x.segment(c->idx, c->dim);
            }
        }
    }
}

SolverBoxBPP::SolverBoxBPP(RigidBodySystem* _rigidBodySystem)
    : Solver(_rigidBodySystem)
    , m_stabilization(250.0f)
    , m_alpha(500.0f)
    , m_beta(125000.0f)
    , m_pivotTolerance(1e-5f)
{
}

void SolverBoxBPP::solve(float h)
{
    auto& joints = m_rigidBodySystem->getJoints();
    auto& contacts = m_rigidBodySystem->getContacts();
    const unsigned int numContacts = contacts.size();
    const unsigned int numJoints = joints.size();
    const unsigned int N = numJoints + numContacts;

    unsigned int dim = 0;
    for (Joint* j : joints)
    {
        j->idx = dim;
        dim += j->dim;
    }
    for (Contact* c : contacts)
    {
        c->idx = dim;
        dim += c->dim;
    }

    if (N > 0)
    {
        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(dim, dim);
        Eigen::VectorXf b = Eigen::VectorXf::Zero(dim);
        Eigen::VectorXf x = Eigen::VectorXf::Zero(dim);

        // Update box bounds.
        Eigen::VectorXf lower = Eigen::VectorXf::Constant(dim, -std::numeric_limits<float>::max());
        Eigen::VectorXf upper = Eigen::VectorXf::Constant(dim, std::numeric_limits<float>::max());

        updateBounds(contacts, lower, upper, m_useOpenMP, m_stabilization);

        // Initialize the index set.
        // All variables are initially set to 'free'.
        Eigen::VectorXi idx = Eigen::VectorXi::Constant(dim, kFree);

        // Ignore variables where lower and upper are zero.
#ifdef USE_OPENMP
        if (m_useOpenMP && dim > 1000) {
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(dim); ++i) {
                static const float tol = 1e-5f;
                if (std::abs(upper(i)) < tol && std::abs(lower(i) < tol))
                    idx(i) = kIgnore;
            }
        } else
#endif
        {
            // Serial version
            for (int i = 0; i < dim; ++i) {
                static const float tol = 1e-5f;
                if (std::abs(upper(i)) < tol && std::abs(lower(i) < tol))
                    idx(i) = kIgnore;
            }
        }

        // Construct the lead matrix and rhs vector.
        buildMatrix(joints, contacts, h, A, m_useOpenMP, m_stabilization, m_alpha, m_beta);
        buildRHS(joints, contacts, h, b, m_useOpenMP, m_stabilization, m_alpha, m_beta);

        // Perform an initial solve to update friction box bounds.
        solvePrincipalSubproblem(A, b, idx, lower, upper, x, m_useOpenMP);
        updateJointsContacts(x, joints, contacts, m_useOpenMP);
        updateBounds(contacts, lower, upper, m_useOpenMP, m_stabilization);
        idx.setConstant(kFree);

        // Block pivoting iterations.
        for (unsigned int iter = 0; iter < m_maxIter; ++iter)
        {
            // Solve the principal sub-problem:
            //    Aff * xf = bf - Aft * xt
            solvePrincipalSubproblem(A, b, idx, lower, upper, x, m_useOpenMP);

            // Compute residual velocity v.
            const Eigen::VectorXf v = A * x - b;

            // Pivot.
            const int numPivots = pivot(idx, x, lower, upper, v, m_useOpenMP);
            if (numPivots == 0)
                break;      // Done
        }

        updateJointsContacts(x, joints, contacts, m_useOpenMP);
        updateBounds(contacts, lower, upper, m_useOpenMP, m_stabilization);
    }
}