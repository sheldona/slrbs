#pragma once

#include "solvers/Solver.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>

namespace slrbs { class SimDataLogger; }
class RigidBodySystem;
class Contact;

/**
 * SolverProximal - Implements a Proximal Gradient Solver for rigid body contact simulation
 *
 * Uses binary data export via SimDataLogger for debug and analysis.
 */
class SolverProximal : public Solver {
public:
    explicit SolverProximal(RigidBodySystem* system);
    ~SolverProximal();

    // Solve the contact impulse update for time step h
    void solve(float h) override;

    // Enable global data export
    void enableDataExport(bool enable);
    // Set base directory for exported logs
    void setExportPath(const std::string& basePath);

    // Toggle individual log streams
    enum class LogType {
        LCP_ERROR,
        RESIDUAL,
        MATRIX_R,
        ACTIVE_CONSTRAINTS,
        PERFORMANCE,
        MATRICES
    };
    void setLogEnabled(LogType type, bool enabled);

private:
    // Initialize internal data logger (called on first solve)
    void initializeDataLogger(const std::string& name);

    bool m_exportEnabled = false;
    std::string m_exportBasePath = "./";
    std::unique_ptr<slrbs::SimDataLogger> m_dataLogger;
    std::unordered_map<LogType,bool> m_logEnabled{
        {LogType::LCP_ERROR, true},
        {LogType::RESIDUAL, true},
        {LogType::MATRIX_R, false},
        {LogType::ACTIVE_CONSTRAINTS, true},
        {LogType::PERFORMANCE, true},
        {LogType::MATRICES, false}
    };

    // Export functions
    void exportMatrices(const std::vector<Eigen::Matrix3f>& A,
                        const std::vector<Contact*>& contacts);
    void exportActiveConstraints(const std::vector<Eigen::Vector3f>& lambdaCandidate,
                                 float mu);
    void exportResidual(float residualNorm);
    void exportLCPError(float lcpError);
    void exportRValues(const std::vector<Eigen::Matrix3f>& R);
    void exportPerformanceData(double solveTimeMs, int iterations);
};