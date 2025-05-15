#include "solvers/SolverProximal.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"
#include "util/Types.h"
#include "logging/SimDataLogger.h"
#include <cfloat>
#include <Eigen/Dense>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace slrbs;

namespace {
// Count active (non‑clamped) constraints
static inline int countActiveConstraints(
    const std::vector<Eigen::Vector3f>& lambda, float mu, bool useOpenMP)
{
    int count = 0;

    if (useOpenMP)
    {
        #ifdef _OPENMP
        int localCounts[omp_get_max_threads()] = {0};
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            #pragma omp for
            for (int i = 0; i < static_cast<int>(lambda.size()); ++i) {
                const auto& l = lambda[i];
                float n = l[0];
                Eigen::Vector2f ft = l.tail<2>();
                float lim = mu * n;
                if (n > 0 && ft.lpNorm<Eigen::Infinity>() < lim)
                    ++localCounts[tid];
            }
        }
        // Combine results from all threads
        for (int i = 0; i < omp_get_max_threads(); ++i) {
            count += localCounts[i];
        }
        #endif
    }
    else
    {
        for (auto& l : lambda) {
            float n = l[0];
            Eigen::Vector2f ft = l.tail<2>();
            float lim = mu * n;
            if (n > 0 && ft.lpNorm<Eigen::Infinity>() < lim) ++count;
        }
    }
    return count;
}

// Compute average LCP error
static inline float computeLCPError(
    const Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b,
    float mu)
{
    Eigen::Vector3f w = A * lambda + b;
    float err = 0;
    Eigen::Vector3f upper(FLT_MAX, mu * lambda[0], mu * lambda[0]);
    Eigen::Vector3f lower(0.0f, -mu * lambda[0], -mu * lambda[0]);
    for (int i = 0; i < 3; ++i) {
        float v = lambda[i];
        float bi = w[i];
        float e_pos = std::max(0.0f, lower[i] - v);
        float e_neg = std::max(0.0f, v - upper[i]);
        float comp  = std::max(0.0f, -bi);
        err += std::max({e_pos, e_neg, comp});
    }
    return err;
}

// Multiply and accumulate JMinv * [f; tau]
static inline void multAndAdd(
    const Contact::JBlock& JMinv,
    const Eigen::Vector3f& f,
    const Eigen::Vector3f& tau,
    float h,
    Eigen::Vector3f& out)
{
    out += h * (
        JMinv.col(0) * f[0]
      + JMinv.col(1) * f[1]
      + JMinv.col(2) * f[2]
      + JMinv.col(3) * tau[0]
      + JMinv.col(4) * tau[1]
      + JMinv.col(5) * tau[2]
    );
}

// Build RHS: b = gamma*phi/h + J*v + h*JMinv*forces
static inline void buildRHS(
    Contact* c, float h, Eigen::Vector3f& b)
{
    float gamma = h * c->k / (h * c->k + c->bias);
    b = gamma * c->phi(0) / h * Eigen::Vector3f::Ones();
    float e = 0.2f;
    if (!c->body0->fixed) {
        Eigen::VectorXf v(6);
        v << c->body0->xdot, c->body0->omega;
        b += (1.0f + e) * (c->J0 * v);
        multAndAdd(c->J0Minv, c->body0->f, c->body0->tau, h, b);
    }
    if (!c->body1->fixed) {
        Eigen::VectorXf v(6);
        v << c->body1->xdot, c->body1->omega;
        b -= (1.0f + e) * (c->J1 * v);
        multAndAdd(c->J1Minv, c->body1->f, c->body1->tau, h, b);
    }
}

// Initialize R adaptively
static inline void initializeR(
    const std::vector<Eigen::Matrix3f>& A,
    std::vector<Eigen::Matrix3f>& R,
    std::vector<Eigen::Matrix3f>& nu,
    bool useOpenMP)
{
    static int frameCount = 0;
    ++frameCount;
    size_t M = A.size();
    R.resize(M);
    nu.resize(M);

    if (useOpenMP)
    {
        #ifdef _OPENMP
        #pragma omp parallel for
        for (int i = 0; i < static_cast<int>(M); ++i) {
            float t = std::clamp(A[i].trace() / 9.0f, 0.1f, 1.0f);
            if (frameCount < 5) {
                R[i]  = 0.5f * t * Eigen::Matrix3f::Identity();
                nu[i] = 0.8f * t * Eigen::Matrix3f::Identity();
            } else if (frameCount < 10) {
                R[i]  = 0.3f * t * Eigen::Matrix3f::Identity();
                nu[i] = 0.9f * t * Eigen::Matrix3f::Identity();
            } else {
                R[i]  = 0.2f * Eigen::Matrix3f::Identity();
                nu[i] = 0.9f * t * Eigen::Matrix3f::Identity();
            }
        }
        #endif
    }
    else
    {
        for (size_t i = 0; i < M; ++i) {
            float t = std::clamp(A[i].trace() / 9.0f, 0.1f, 1.0f);
            if (frameCount < 5) {
                R[i]  = 0.5f * t * Eigen::Matrix3f::Identity();
                nu[i] = 0.8f * t * Eigen::Matrix3f::Identity();
            } else if (frameCount < 10) {
                R[i]  = 0.3f * t * Eigen::Matrix3f::Identity();
                nu[i] = 0.9f * t * Eigen::Matrix3f::Identity();
            } else {
                R[i]  = 0.2f * Eigen::Matrix3f::Identity();
                nu[i] = 0.9f * t * Eigen::Matrix3f::Identity();
            }
        }
    }
}

// Proximity operators
static inline void proxN(const Eigen::Vector3f& z, Eigen::Vector3f& x) {
    x[0] = std::max(0.0f, z[0]);
}
static inline void proxF(const Eigen::Vector3f& z,
                         Eigen::Vector3f& x, float mu)
{
    x[1] = 0;
    x[2] = 0;
    if (x[0] <= 0) return;
    float lim = mu * x[0];
    x[1] = std::clamp(z[1], -lim, lim);
    x[2] = std::clamp(z[2], -lim, lim);
}
// Update w += MinvJ^T * lambda
static inline void updateW(
    Eigen::VectorXf& w,
    const Contact::JBlockTranspose& MinvJT,
    const Eigen::Vector3f& lambda)
{
    w += MinvJT * lambda;
}
} // namespace

// -----------------------------------------------------------------------------
SolverProximal::SolverProximal(RigidBodySystem* sys)
  : Solver(sys)
{}

SolverProximal::~SolverProximal() {}

void SolverProximal::enableDataExport(bool en) { m_exportEnabled = en; }
void SolverProximal::setExportPath(const std::string& p) {
    m_exportBasePath = p;
    std::filesystem::create_directories(p);
}
void SolverProximal::setLogEnabled(LogType t, bool en) {
    m_logEnabled[t] = en;
}

void SolverProximal::initializeDataLogger(const std::string& name) {
    m_dataLogger = std::make_unique<slrbs::SimDataLogger>();
    m_dataLogger->initialize(m_exportBasePath);
    m_dataLogger->startLogging(name, /*append=*/false);
}

// Export implementations
void SolverProximal::exportMatrices(const std::vector<Eigen::Matrix3f>& A,
                                    const std::vector<Contact*>& contacts)
{
    if (!m_exportEnabled || !m_logEnabled[LogType::MATRICES] || !m_dataLogger)
        return;
    for (size_t i = 0; i < A.size(); ++i) {
        m_dataLogger->logMatrix3f("A_" + std::to_string(i), A[i]);
        Contact* c = contacts[i];
        m_dataLogger->logMatrix("J0_" + std::to_string(i), c->J0);
        m_dataLogger->logMatrix("J1_" + std::to_string(i), c->J1);
        m_dataLogger->logMatrix("J0Minv_" + std::to_string(i), c->J0Minv);
        m_dataLogger->logMatrix("J1Minv_" + std::to_string(i), c->J1Minv);
        m_dataLogger->logScalar("Phi_" + std::to_string(i), c->phi(0));
        m_dataLogger->logScalar("Mu_" + std::to_string(i), c->mu);
    }
}

void SolverProximal::exportActiveConstraints(const std::vector<Eigen::Vector3f>& lam,
                                             float mu)
{
    if (!m_exportEnabled || !m_logEnabled[LogType::ACTIVE_CONSTRAINTS] || !m_dataLogger)
        return;
    int active = countActiveConstraints(lam, mu, m_useOpenMP);
    int total  = static_cast<int>(lam.size()) * 3;
    m_dataLogger->logInt("Active", active);
    m_dataLogger->logInt("Inactive", total - active);
    m_dataLogger->logInt("Total", total);
}

void SolverProximal::exportResidual(float r) {
    if (m_exportEnabled && m_logEnabled[LogType::RESIDUAL] && m_dataLogger)
        m_dataLogger->logScalar("Residual", r);
}

void SolverProximal::exportLCPError(float e) {
    if (m_exportEnabled && m_logEnabled[LogType::LCP_ERROR] && m_dataLogger)
        m_dataLogger->logScalar("LCPError", e);
}

void SolverProximal::exportRValues(const std::vector<Eigen::Matrix3f>& R) {
    if (!m_exportEnabled || !m_logEnabled[LogType::MATRIX_R] || !m_dataLogger)
        return;
    for (size_t i = 0; i < R.size(); ++i)
        m_dataLogger->logMatrix3f("R_" + std::to_string(i), R[i]);
}

void SolverProximal::exportPerformanceData(double t, int it) {
    if (m_exportEnabled && m_logEnabled[LogType::PERFORMANCE] && m_dataLogger) {
        m_dataLogger->logScalar("SolveTimeMs", static_cast<float>(t));
        m_dataLogger->logInt("Iterations", it);
    }
}

void SolverProximal::solve(float h) {
    auto start = std::chrono::high_resolution_clock::now();
    if (m_exportEnabled && !m_dataLogger)
        initializeDataLogger("proxSolver");
    if (m_exportEnabled)
        m_dataLogger->beginFrame();

    auto& C = m_rigidBodySystem->getContacts();
    auto& B = m_rigidBodySystem->getBodies();
    int M = static_cast<int>(C.size());
    int N = static_cast<int>(B.size());
    std::unordered_map<RigidBody*,int> idx;
    idx.reserve(N);
    for (int i = 0; i < N; ++i)
        idx[B[i]] = i;

    std::vector<Eigen::Matrix3f> A(M);
    std::vector<Eigen::Vector3f> lam(M), b(M);
    std::vector<float> mu(M);

    if (m_useOpenMP)
    {
        #ifdef _OPENMP
        #pragma omp parallel for
        for (int i = 0; i < M; ++i) {
            auto* c = C[i];
            mu[i] = c->mu;
            A[i].setZero();
            float cmix = 1.0f / (h * c->k + c->bias);
            A[i].diagonal().array() = cmix;
            if (!c->body0->fixed) A[i] += c->J0Minv * c->J0.transpose();
            if (!c->body1->fixed) A[i] += c->J1Minv * c->J1.transpose();
            buildRHS(c, h, b[i]);
            c->lambda.setZero();
        }
        #endif
    }
    else
    {
        for (int i = 0; i < M; ++i) {
            auto* c = C[i];
            mu[i] = c->mu;
            A[i].setZero();
            float cmix = 1.0f / (h * c->k + c->bias);
            A[i].diagonal().array() = cmix;
            if (!c->body0->fixed) A[i] += c->J0Minv * c->J0.transpose();
            if (!c->body1->fixed) A[i] += c->J1Minv * c->J1.transpose();
            buildRHS(c, h, b[i]);
            c->lambda.setZero();
        }
    }

    if (m_exportEnabled && m_logEnabled[LogType::MATRICES])
        exportMatrices(A, C);

    std::vector<Eigen::Vector3f> lamCand(M), resid(M);
    std::vector<Eigen::Matrix3f> R, nu;
    std::vector<Eigen::VectorXf> w(N, Eigen::VectorXf::Zero(6));
    initializeR(A, R, nu, m_useOpenMP);

    float absTol = m_absTolerance, relTol = m_relTolerance;
    int it = 0;
    float last = 0, cur = 0;
    for (it = 0; it < m_maxIter; ++it) {
        // Reset w vector
        if (m_useOpenMP)
        {
            #ifdef _OPENMP
            #pragma omp parallel for
            for (int i = 0; i < N; ++i) {
                w[i].setZero();
            }
            #endif
        }
        else
        {
            for (auto& wv : w) wv.setZero();
        }

        // Accumulate initial w
        if (m_useOpenMP)
        {
            #ifdef _OPENMP
            #pragma omp parallel for
            for (int i = 0; i < M; ++i) {
                auto* c = C[i];
                #pragma omp critical
                {
                    updateW(w[idx[c->body0]], c->MinvJ0T, c->lambda);
                    updateW(w[idx[c->body1]], c->MinvJ1T, c->lambda);
                }
            }
            #endif
        }
        else
        {
            for (int i = 0; i < M; ++i) {
                auto* c = C[i];
                updateW(w[idx[c->body0]], c->MinvJ0T, c->lambda);
                updateW(w[idx[c->body1]], c->MinvJ1T, c->lambda);
            }
        }

        cur = 0;
        if (m_useOpenMP)
        {
            #ifdef _OPENMP
            float local_max[omp_get_max_threads()] = {0};

            #pragma omp parallel
            {
                int tid = omp_get_thread_num();

                #pragma omp for
                for (int i = 0; i < M; ++i) {
                    auto* c = C[i];
                    int i0 = idx[c->body0], i1 = idx[c->body1];
                    Eigen::Vector3f z = c->lambda
                        - R[i] * (c->J0 * w[i0] + c->J1 * w[i1] + b[i]);
                    lamCand[i] = c->lambda;
                    proxN(z, lamCand[i]);
                    proxF(z, lamCand[i], mu[i]);
                    resid[i] = lamCand[i] - c->lambda;
                    local_max[tid] = std::max(local_max[tid], resid[i].lpNorm<Eigen::Infinity>());

                    #pragma omp critical
                    {
                        updateW(w[i0], c->MinvJ0T, resid[i]);
                        updateW(w[i1], c->MinvJ1T, resid[i]);
                    }
                }
            }

            // Combine max results from all threads
            for (int i = 0; i < omp_get_max_threads(); ++i) {
                cur = std::max(cur, local_max[i]);
            }
            #endif
        }
        else
        {
            for (int i = 0; i < M; ++i) {
                auto* c = C[i];
                int i0 = idx[c->body0], i1 = idx[c->body1];
                Eigen::Vector3f z = c->lambda
                    - R[i] * (c->J0 * w[i0] + c->J1 * w[i1] + b[i]);
                lamCand[i] = c->lambda;
                proxN(z, lamCand[i]);
                proxF(z, lamCand[i], mu[i]);
                resid[i] = lamCand[i] - c->lambda;
                cur = std::max(cur, resid[i].lpNorm<Eigen::Infinity>());
                updateW(w[i0], c->MinvJ0T, resid[i]);
                updateW(w[i1], c->MinvJ1T, resid[i]);
            }
        }

        if (m_exportEnabled && m_logEnabled[LogType::ACTIVE_CONSTRAINTS])
            exportActiveConstraints(lamCand, mu[0]);
        if (m_exportEnabled && m_logEnabled[LogType::RESIDUAL])
            exportResidual(cur);
        if (cur < absTol) break;
        if (it > 0 && std::fabs(cur - last) < relTol * last) break;
        last = cur;

        // Update lambda values
        if (m_useOpenMP)
        {
            #ifdef _OPENMP
            #pragma omp parallel for
            for (int i = 0; i < M; ++i) {
                C[i]->lambda = lamCand[i];
            }
            #endif
        }
        else
        {
            for (int i = 0; i < M; ++i)
                C[i]->lambda = lamCand[i];
        }

        if (it > 0 && cur > last) {
            if (m_useOpenMP)
            {
                #ifdef _OPENMP
                #pragma omp parallel for
                for (int i = 0; i < static_cast<int>(R.size()); ++i) {
                    R[i].diagonal().setConstant(0.2f);
                }
                #endif
            }
            else
            {
                for (auto& Ri : R) Ri.diagonal().setConstant(0.2f);
            }
        } else if (m_exportEnabled && m_logEnabled[LogType::MATRIX_R] && (it % 10) == 0) {
            exportRValues(R);
        }
    }

    // Compute final LCP error
    float totalErr = 0;
    if (m_useOpenMP)
    {
        #ifdef _OPENMP
        float local_errors[omp_get_max_threads()] = {0};

        #pragma omp parallel
        {
            int tid = omp_get_thread_num();

            #pragma omp for
            for (int i = 0; i < M; ++i) {
                local_errors[tid] += computeLCPError(C[i]->lambda, A[i], b[i], C[i]->mu);
            }
        }

        // Combine error results from all threads
        for (int i = 0; i < omp_get_max_threads(); ++i) {
            totalErr += local_errors[i];
        }
        #endif
    }
    else
    {
        for (int i = 0; i < M; ++i)
            totalErr += computeLCPError(C[i]->lambda, A[i], b[i], C[i]->mu);
    }

    float avgErr = M ? totalErr / M : 0.0f;
    if (m_exportEnabled && m_logEnabled[LogType::LCP_ERROR])
        exportLCPError(avgErr);

    auto end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
    if (m_exportEnabled && m_logEnabled[LogType::PERFORMANCE])
        exportPerformanceData(elapsed, it);
    if (m_exportEnabled)
        m_dataLogger->endFrame();

    std::cout << "[SolverProximal] iters=" << it
              << " avgErr=" << avgErr << std::endl;
}