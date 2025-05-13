#include "logging/SimDataLogger.h"
#include <fstream>
#include <filesystem>

namespace slrbs {

    SimDataLogger::SimDataLogger()
      : m_frame(0)
    {}

    SimDataLogger::~SimDataLogger() {
        if (m_stream.is_open())
            m_stream.close();
    }

    void SimDataLogger::initialize(const std::string& basePath) {
        m_basePath = basePath;
        std::filesystem::create_directories(basePath);
    }

    void SimDataLogger::startLogging(const std::string& name, bool append) {
        if (m_stream.is_open())
            m_stream.close();
        std::string filename = m_basePath + "/" + name + ".csv";
        m_stream.open(filename, append ? std::ios::app : std::ios::trunc);
        // Write CSV header
        m_stream << "frame,label,values\n";
    }

    void SimDataLogger::beginFrame() {
        ++m_frame;
    }

    void SimDataLogger::endFrame() {
        if (m_stream.is_open())
            m_stream.flush();
    }

    void SimDataLogger::logMatrix3f(const std::string& label, const Eigen::Matrix3f& mat) {
        if (!m_stream.is_open()) return;
        // flatten 3x3 matrix into one CSV row
        m_stream << m_frame << "," << label;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m_stream << "," << mat(i,j);
        m_stream << "\n";
    }

    void SimDataLogger::logMatrix(const std::string& label, const Eigen::MatrixXf& mat) {
        if (!m_stream.is_open()) return;
        // each row of mat becomes its own entry
        for (int i = 0; i < mat.rows(); ++i) {
            m_stream << m_frame << "," << label;
            for (int j = 0; j < mat.cols(); ++j)
                m_stream << "," << mat(i,j);
            m_stream << "\n";
        }
    }

    void SimDataLogger::logScalar(const std::string& label, float value) {
        if (!m_stream.is_open()) return;
        m_stream << m_frame << "," << label << "," << value << "\n";
    }

    void SimDataLogger::logInt(const std::string& label, int value) {
        if (!m_stream.is_open()) return;
        m_stream << m_frame << "," << label << "," << value << "\n";
    }

} // namespace slrbs