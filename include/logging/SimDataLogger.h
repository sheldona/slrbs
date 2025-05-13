#pragma once

#include <string>
#include <Eigen/Dense>
#include <fstream>

namespace slrbs {

/**
 * SimDataLogger
 *
 * Simple CSV-based logger for simulation data. Logs frames of scalar,
 * integer, and matrix values into a single CSV file.
 */
class SimDataLogger {
public:
    /**
     * Construct an uninitialized logger.
     */
    SimDataLogger();

    /**
     * Close any open streams.
     */
    ~SimDataLogger();

    /**
     * Initialize the logger with a base output directory. Creates directories as needed.
     * @param basePath  Directory to write CSV files into
     */
    void initialize(const std::string& basePath);

    /**
     * Start logging under a given simulation name. Opens (and optionally appends to) a CSV file.
     * @param name    Base name of the CSV file ("<basePath>/<name>.csv")
     * @param append  If true, existing file is appended; otherwise truncated
     */
    void startLogging(const std::string& name, bool append);

    /**
     * Mark the beginning of a new frame (increments internal frame counter).
     */
    void beginFrame();

    /**
     * Flush any pending data for the current frame.
     */
    void endFrame();

    /**
     * Log a fixed-size 3x3 matrix (floats).
     * @param label  Identifier for this matrix
     * @param mat    3x3 float matrix to log
     */
    void logMatrix3f(const std::string& label, const Eigen::Matrix3f& mat);

    /**
     * Log a dynamic-size float matrix. Each row produces one CSV line.
     * @param label  Identifier for this matrix
     * @param mat    Dynamic-size float matrix to log
     */
    void logMatrix(const std::string& label, const Eigen::MatrixXf& mat);

    /**
     * Log a single float scalar value.
     * @param label  Identifier for this scalar
     * @param value  Float value to log
     */
    void logScalar(const std::string& label, float value);

    /**
     * Log a single integer value.
     * @param label  Identifier for this integer
     * @param value  Integer value to log
     */
    void logInt(const std::string& label, int value);

private:
    std::string m_basePath;  ///< Base output directory
    std::ofstream m_stream;  ///< CSV output stream
    int m_frame;             ///< Current frame counter (starts at 0)
};

} // namespace slrbs
