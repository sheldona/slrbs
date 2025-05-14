#pragma once

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <Eigen/Dense>

// Forward declarations
class RigidBody;
class RigidBodySystem;

/**
 * Utility class to track and visualize how many times each face of a mesh is hit during collisions.
 */
class FaceContactTracker {
public:
    // Store hit count for each face in a mesh
    struct FaceHitData {
        std::vector<int> hitCounts;                     // Hit count per face
        std::vector<Eigen::Vector3f> tangentDirections; // Tangent direction of movement per face
        int maxHits;                                    // Maximum hit count (for normalization)

        FaceHitData() : maxHits(0) {}

        void reset() {
            std::fill(hitCounts.begin(), hitCounts.end(), 0);
            for (auto& tangent : tangentDirections) {
                tangent.setZero();
            }
            maxHits = 0;
        }
    };

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

    // Reset all tracking data
    static void reset();

    // Update visualization in Polyscope
    static void updateVisualization(RigidBodySystem* system);

    // Enable/disable visualization
    static void setVisualizationEnabled(bool enabled) { s_visualizationEnabled = enabled; }
    static bool isVisualizationEnabled() { return s_visualizationEnabled; }

    // Enable/disable tangent visualization
    static void setTangentVisualizationEnabled(bool enabled) { s_tangentVisualizationEnabled = enabled; }
    static bool isTangentVisualizationEnabled() { return s_tangentVisualizationEnabled; }

private:
    // Maps body ID to face hit data
    static std::map<int, FaceHitData> s_bodyFaceHits;
    static bool s_visualizationEnabled;
    static bool s_tangentVisualizationEnabled;
};