#pragma once

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <functional>

// Forward declarations
class RigidBody;
class RigidBodySystem;

/**
 * Utility class to track and visualize how many times each face of a mesh is hit during collisions.
 */
class FaceContactTracker {
public:
    // Visualization modes
    enum class VisualizationMode {
        NONE,               // No visualization
        COLOR_GRADIENT,     // Red gradient based on hit count
        HEAT_MAP,           // Heat map coloring (blue to red)
        CUSTOM_COLORMAP     // User-defined color map
    };

    // Tangent visualization modes
    enum class TangentVisualizationMode {
        NONE,               // No tangent visualization
        ARROWS,             // Vector arrows
        STREAMLINES,        // Streamline visualization
        POINTS              // Colored points
    };

    // Visualization parameters
    struct VisualizationParams {
        VisualizationMode mode = VisualizationMode::COLOR_GRADIENT;
        TangentVisualizationMode tangentMode = TangentVisualizationMode::ARROWS;
        int hitCountThreshold = 50;           // Maximum hit count for normalization
        float visualScale = 1.0f;             // Scale for visualizations
        float pointRadius = 0.005f;           // Radius for points
        float vectorScale = 1.0f;             // Scale for vectors/tangents
        bool blendWithOriginalColor = true;   // Blend with original mesh color
        bool showMaxHitLabels = false;        // Show hit count labels

        // Color settings
        Eigen::Vector3f hitColor = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Red for hits
        Eigen::Vector3f tangentColor = Eigen::Vector3f(0.0f, 0.8f, 0.2f); // Green for tangents
        Eigen::Vector3f pointColor = Eigen::Vector3f(0.8f, 0.2f, 0.2f); // Red for points

        // Hit count decay
        bool enableHitDecay = false;          // Enable hit count decay over time
        float hitDecayRate = 0.05f;           // Rate of hit count decay per second

        // Custom visualization callback
        std::function<void(RigidBody*, const std::vector<int>&)> customVisCallback = nullptr;
    };

    // Store hit count for each face in a mesh
    struct FaceHitData {
        std::vector<int> hitCounts;                     // Hit count per face
        std::vector<Eigen::Vector3f> tangentDirections; // Tangent direction of movement per face
        std::vector<float> impactVelocities;            // Impact velocity magnitudes
        std::vector<float> impactTimes;                 // Time of last impact
        int maxHits;                                    // Maximum hit count (for normalization)
        float totalImpactEnergy;                        // Accumulated impact energy

        FaceHitData() : maxHits(0), totalImpactEnergy(0.0f) {}

        void reset() {
            std::fill(hitCounts.begin(), hitCounts.end(), 0);
            for (auto& tangent : tangentDirections) {
                tangent.setZero();
            }
            std::fill(impactVelocities.begin(), impactVelocities.end(), 0.0f);
            std::fill(impactTimes.begin(), impactTimes.end(), 0.0f);
            maxHits = 0;
            totalImpactEnergy = 0.0f;
        }
    };

    // Logging options
    enum class LogOption {
        HIT_COUNTS,
        TANGENT_DIRECTIONS,
        IMPACT_VELOCITIES,
        IMPACT_ENERGY,
        IMPACT_TIMESTAMPS
    };

    // Logging params
    struct LoggingParams {
        bool enabled = false;
        std::string logPath = "contact_logs";
        bool autoFlush = true;
        float logInterval = 1.0f;  // Seconds between logs
        bool separateFiles = true; // One file per body
        bool appendTimestamp = true;
        std::map<LogOption, bool> options = {
            {LogOption::HIT_COUNTS, true},
            {LogOption::TANGENT_DIRECTIONS, false},
            {LogOption::IMPACT_VELOCITIES, false},
            {LogOption::IMPACT_ENERGY, false},
            {LogOption::IMPACT_TIMESTAMPS, false}
        };
    };

    // Static member access methods
    static void setLogParams(const LoggingParams& params);
    static LoggingParams& getLogParams();

    static void setVisParams(const VisualizationParams& params);
    static VisualizationParams& getVisParams();

    static void setLoggingEnabled(bool enabled);
    static void setLogPath(const std::string& path);
    static bool isLogOptionEnabled(LogOption option);
    static void setLogOptionEnabled(LogOption option, bool enabled);

    // Initialize the tracking system
    static void initialize();

    // Clean up all resources
    static void shutdown();

    // Initialize tracking for a specific rigid body
    static void initializeForBody(RigidBody* body);

    // Record a hit on a specific face of a body
    static void recordHit(RigidBody* body, int faceIndex);

    // Record a hit with tangent direction
    static void recordHit(RigidBody* body, int faceIndex, const Eigen::Vector3f& tangent);

    // Record a hit with impact data
    static void recordHit(RigidBody* body, int faceIndex, const Eigen::Vector3f& tangent,
                         float impactVelocity, float time);

    // Decay hit counts over time
    static void update(float dt);

    // Reset all tracking data
    static void reset();

    // Update visualization in Polyscope
    static void updateVisualization(RigidBodySystem* system);

    // Enable/disable visualization
    static void setVisualizationEnabled(bool enabled);
    static bool isVisualizationEnabled();

    // Enable/disable tangent visualization
    static void setTangentVisualizationEnabled(bool enabled);
    static bool isTangentVisualizationEnabled();

    // Set visualization mode
    static void setVisualizationMode(VisualizationMode mode);
    static void setTangentVisualizationMode(TangentVisualizationMode mode);

    // Log current hit data
    static void flushLogs();

private:
    // Maps body ID to face hit data
    static std::map<int, FaceHitData> s_bodyFaceHits;

    // Visualization and logging parameters
    static VisualizationParams s_visParams;
    static LoggingParams s_logParams;

    // Visualization state
    static bool s_visualizationEnabled;
    static bool s_tangentVisualizationEnabled;

    // Logging state
    static float s_timeSinceLastLog;
    static bool s_logDirCreated;

    // Initialize logging directory
    static void initializeLogging();

    // Internal visualization helpers
    static void visualizeHitGradient(RigidBody* body, const FaceHitData& hitData);
    static void visualizeHitHeatmap(RigidBody* body, const FaceHitData& hitData);
    static void visualizeTangentArrows(RigidBody* body, const FaceHitData& hitData);
    static void visualizeTangentStreamlines(RigidBody* body, const FaceHitData& hitData);
};