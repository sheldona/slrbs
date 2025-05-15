#include "contact/FaceContactTracker.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "collision/Geometry.h"
#include <algorithm>
#include <iostream>
#include <filesystem>
#include <fstream>

// Static member initialization
std::map<int, FaceContactTracker::FaceHitData> FaceContactTracker::s_bodyFaceHits;
FaceContactTracker::VisualizationParams FaceContactTracker::s_visParams;
FaceContactTracker::LoggingParams FaceContactTracker::s_logParams;
bool FaceContactTracker::s_visualizationEnabled = false;
bool FaceContactTracker::s_tangentVisualizationEnabled = false;
float FaceContactTracker::s_timeSinceLastLog = 0.0f;
bool FaceContactTracker::s_logDirCreated = false;

void FaceContactTracker::initialize() {
    s_bodyFaceHits.clear();
    s_visualizationEnabled = false;
    s_tangentVisualizationEnabled = false;
    s_timeSinceLastLog = 0.0f;
    s_logDirCreated = false;
    std::cout << "Face contact tracker initialized" << std::endl;
}

void FaceContactTracker::shutdown() {
    s_bodyFaceHits.clear();
}

void FaceContactTracker::initializeForBody(RigidBody* body) {
    if (!body || !body->mesh) return;

    // Estimate face count based on the geometry type
    // This avoids the need to access private members of polyscope::SurfaceMesh
    size_t estimatedFaces = 12; // Default for a box

    if (body->geometry) {
        switch (body->geometry->getType()) {
            case kBox:
                estimatedFaces = 12; // 12 triangles for a box (2 per face)
                break;
            case kSphere:
                estimatedFaces = 80; // Typical sphere face count
                break;
            case kCylinder:
                estimatedFaces = 80; // Typical cylinder face count
                break;
            case kPlane:
                estimatedFaces = 2;  // Plane has 2 triangles typically
                break;
            default:
                estimatedFaces = 20; // Reasonable default
                break;
        }
    }

    FaceHitData data;
    data.hitCounts.resize(estimatedFaces, 0);
    data.tangentDirections.resize(estimatedFaces, Eigen::Vector3f::Zero());
    data.impactVelocities.resize(estimatedFaces, 0.0f);
    data.impactTimes.resize(estimatedFaces, 0.0f);
    data.maxHits = 0;
    data.totalImpactEnergy = 0.0f;
    s_bodyFaceHits[body->id] = data;

    std::cout << "Initialized tracking for body " << body->id
              << " with estimated " << estimatedFaces << " faces" << std::endl;
}

void FaceContactTracker::recordHit(RigidBody* body, int faceIndex) {
    if (!body || faceIndex < 0) return;

    auto it = s_bodyFaceHits.find(body->id);
    if (it == s_bodyFaceHits.end()) {
        // Initialize on first hit if not already done
        initializeForBody(body);
        it = s_bodyFaceHits.find(body->id);
        if (it == s_bodyFaceHits.end()) return;
    }

    auto& data = it->second;
    if (faceIndex < static_cast<int>(data.hitCounts.size())) {
        data.hitCounts[faceIndex]++;
        data.maxHits = std::max(data.maxHits, data.hitCounts[faceIndex]);
    }
}

void FaceContactTracker::recordHit(RigidBody* body, int faceIndex, const Eigen::Vector3f& tangent) {
    if (!body || faceIndex < 0) return;

    auto it = s_bodyFaceHits.find(body->id);
    if (it == s_bodyFaceHits.end()) {
        // Initialize on first hit if not already done
        initializeForBody(body);
        it = s_bodyFaceHits.find(body->id);
        if (it == s_bodyFaceHits.end()) return;
    }

    auto& data = it->second;
    if (faceIndex < static_cast<int>(data.hitCounts.size())) {
        data.hitCounts[faceIndex]++;

        // Store and update tangent direction
        if (tangent.norm() > 1e-6f) {
            if (data.tangentDirections[faceIndex].norm() < 1e-6f) {
                // First hit, just store the tangent
                data.tangentDirections[faceIndex] = tangent.normalized();
            } else {
                // Average with existing tangent (weighted blend)
                float alpha = 0.3f; // Blend factor - new hits have 30% influence
                data.tangentDirections[faceIndex] =
                    ((1.0f - alpha) * data.tangentDirections[faceIndex] +
                     alpha * tangent.normalized()).normalized();
            }
        }

        data.maxHits = std::max(data.maxHits, data.hitCounts[faceIndex]);
    }
}

void FaceContactTracker::recordHit(RigidBody* body, int faceIndex, const Eigen::Vector3f& tangent,
                                  float impactVelocity, float time) {
    if (!body || faceIndex < 0) return;

    auto it = s_bodyFaceHits.find(body->id);
    if (it == s_bodyFaceHits.end()) {
        // Initialize on first hit if not already done
        initializeForBody(body);
        it = s_bodyFaceHits.find(body->id);
        if (it == s_bodyFaceHits.end()) return;
    }

    auto& data = it->second;
    if (faceIndex < static_cast<int>(data.hitCounts.size())) {
        data.hitCounts[faceIndex]++;

        // Store and update tangent direction
        if (tangent.norm() > 1e-6f) {
            if (data.tangentDirections[faceIndex].norm() < 1e-6f) {
                // First hit, just store the tangent
                data.tangentDirections[faceIndex] = tangent.normalized();
            } else {
                // Average with existing tangent (weighted blend)
                float alpha = 0.3f; // Blend factor - new hits have 30% influence
                data.tangentDirections[faceIndex] =
                    ((1.0f - alpha) * data.tangentDirections[faceIndex] +
                     alpha * tangent.normalized()).normalized();
            }
        }

        // Store impact velocity and time
        data.impactVelocities[faceIndex] = impactVelocity;
        data.impactTimes[faceIndex] = time;

        // Update total impact energy (approximation)
        data.totalImpactEnergy += 0.5f * impactVelocity * impactVelocity;

        data.maxHits = std::max(data.maxHits, data.hitCounts[faceIndex]);
    }
}

void FaceContactTracker::reset() {
    for (auto& [bodyId, data] : s_bodyFaceHits) {
        data.reset();
    }
}

void FaceContactTracker::update(float dt) {
    if (s_visParams.enableHitDecay) {
        for (auto& [bodyId, data] : s_bodyFaceHits) {
            // Apply decay to hit counts
            for (size_t i = 0; i < data.hitCounts.size(); i++) {
                if (data.hitCounts[i] > 0) {
                    float decayAmount = s_visParams.hitDecayRate * dt;
                    data.hitCounts[i] = std::max(0, data.hitCounts[i] - static_cast<int>(decayAmount));
                }
            }

            // Update max hits
            data.maxHits = 0;
            for (int hitCount : data.hitCounts) {
                data.maxHits = std::max(data.maxHits, hitCount);
            }
        }
    }

    // Update logging timer
    if (s_logParams.enabled) {
        s_timeSinceLastLog += dt;
        if (s_timeSinceLastLog >= s_logParams.logInterval && s_logParams.autoFlush) {
            flushLogs();
            s_timeSinceLastLog = 0.0f;
        }
    }
}

void FaceContactTracker::setVisualizationEnabled(bool enabled) {
    s_visualizationEnabled = enabled;
}

bool FaceContactTracker::isVisualizationEnabled() {
    return s_visualizationEnabled;
}

void FaceContactTracker::setTangentVisualizationEnabled(bool enabled) {
    s_tangentVisualizationEnabled = enabled;
}

bool FaceContactTracker::isTangentVisualizationEnabled() {
    return s_tangentVisualizationEnabled;
}

void FaceContactTracker::setVisualizationMode(VisualizationMode mode) {
    s_visParams.mode = mode;
}

void FaceContactTracker::setTangentVisualizationMode(TangentVisualizationMode mode) {
    s_visParams.tangentMode = mode;
}

void FaceContactTracker::setVisParams(const VisualizationParams& params) {
    s_visParams = params;
}

FaceContactTracker::VisualizationParams& FaceContactTracker::getVisParams() {
    return s_visParams;
}

void FaceContactTracker::setLogParams(const LoggingParams& params) {
    s_logParams = params;
    if (params.enabled && !s_logDirCreated) {
        initializeLogging();
    }
}

FaceContactTracker::LoggingParams& FaceContactTracker::getLogParams() {
    return s_logParams;
}

void FaceContactTracker::setLoggingEnabled(bool enabled) {
    s_logParams.enabled = enabled;
    if (enabled && !s_logDirCreated) {
        initializeLogging();
    }
}

void FaceContactTracker::setLogPath(const std::string& path) {
    s_logParams.logPath = path;
    // Reset log directory creation flag so it will be recreated
    s_logDirCreated = false;
}

bool FaceContactTracker::isLogOptionEnabled(LogOption option) {
    return s_logParams.options.count(option) > 0 && s_logParams.options[option];
}

void FaceContactTracker::setLogOptionEnabled(LogOption option, bool enabled) {
    s_logParams.options[option] = enabled;
}

void FaceContactTracker::initializeLogging() {
    if (s_logDirCreated) return;

    try {
        // Create log directory if it doesn't exist
        std::filesystem::create_directories(s_logParams.logPath);
        s_logDirCreated = true;
        std::cout << "Contact logging initialized at: " << s_logParams.logPath << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create log directory: " << e.what() << std::endl;
    }
}

void FaceContactTracker::flushLogs() {
    if (!s_logParams.enabled || s_bodyFaceHits.empty()) return;

    if (!s_logDirCreated) {
        initializeLogging();
        if (!s_logDirCreated) return; // Failed to create log directory
    }

    // Get current timestamp if appending
    std::string timestamp;
    if (s_logParams.appendTimestamp) {
        // Simple timestamp format: YYYYMMDD_HHMMSS
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        char buffer[20];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&time_t_now));
        timestamp = "_" + std::string(buffer);
    }

    // Log each body's data
    for (const auto& [bodyId, data] : s_bodyFaceHits) {
        if (data.maxHits == 0) continue; // Skip bodies with no hits

        std::string filename;
        if (s_logParams.separateFiles) {
            filename = s_logParams.logPath + "/body_" + std::to_string(bodyId) + timestamp + ".csv";
        } else {
            filename = s_logParams.logPath + "/contact_log" + timestamp + ".csv";
        }

        std::ofstream file(filename, std::ios::out | (s_logParams.separateFiles ? std::ios::trunc : std::ios::app));
        if (!file.is_open()) {
            std::cerr << "Failed to open log file: " << filename << std::endl;
            continue;
        }

        // Write header if this is a new file
        if (file.tellp() == 0) {
            file << "BodyID,FaceIndex";
            if (isLogOptionEnabled(LogOption::HIT_COUNTS)) file << ",HitCount";
            if (isLogOptionEnabled(LogOption::TANGENT_DIRECTIONS)) file << ",TangentX,TangentY,TangentZ";
            if (isLogOptionEnabled(LogOption::IMPACT_VELOCITIES)) file << ",ImpactVelocity";
            if (isLogOptionEnabled(LogOption::IMPACT_ENERGY)) file << ",ImpactEnergy";
            if (isLogOptionEnabled(LogOption::IMPACT_TIMESTAMPS)) file << ",LastImpactTime";
            file << "\n";
        }

        // Write data for each face
        for (size_t i = 0; i < data.hitCounts.size(); i++) {
            if (data.hitCounts[i] == 0) continue; // Skip faces with no hits

            file << bodyId << "," << i;
            if (isLogOptionEnabled(LogOption::HIT_COUNTS)) file << "," << data.hitCounts[i];
            if (isLogOptionEnabled(LogOption::TANGENT_DIRECTIONS)) {
                file << "," << data.tangentDirections[i].x()
                     << "," << data.tangentDirections[i].y()
                     << "," << data.tangentDirections[i].z();
            }
            if (isLogOptionEnabled(LogOption::IMPACT_VELOCITIES)) file << "," << data.impactVelocities[i];
            if (isLogOptionEnabled(LogOption::IMPACT_ENERGY)) {
                float energy = 0.5f * data.impactVelocities[i] * data.impactVelocities[i];
                file << "," << energy;
            }
            if (isLogOptionEnabled(LogOption::IMPACT_TIMESTAMPS)) file << "," << data.impactTimes[i];
            file << "\n";
        }

        file.close();
    }

    std::cout << "Contact logs flushed to " << s_logParams.logPath << std::endl;
}

void FaceContactTracker::visualizeHitGradient(RigidBody* body, const FaceHitData& hitData) {
    // Implemented in updateVisualization for now
    // This is a placeholder for future implementation
}

void FaceContactTracker::visualizeHitHeatmap(RigidBody* body, const FaceHitData& hitData) {
    // Implemented in updateVisualization for now
    // This is a placeholder for future implementation
}

void FaceContactTracker::visualizeTangentArrows(RigidBody* body, const FaceHitData& hitData) {
    // Implemented in updateVisualization for now
    // This is a placeholder for future implementation
}

void FaceContactTracker::visualizeTangentStreamlines(RigidBody* body, const FaceHitData& hitData) {
    // Implemented in updateVisualization for now
    // This is a placeholder for future implementation
}

void FaceContactTracker::updateVisualization(RigidBodySystem* system) {
    if (!s_visualizationEnabled || !system) return;

    // Get bodies from the provided system
    auto& bodies = system->getBodies();

    // Standard hit count visualization
    for (auto& [bodyId, hitData] : s_bodyFaceHits) {
        // Find the corresponding rigid body
        RigidBody* body = nullptr;
        for (auto* b : bodies) {
            if (b->id == bodyId) {
                body = b;
                break;
            }
        }

        if (!body || !body->mesh || hitData.maxHits == 0) continue;

        // Since we can't directly add face scalar fields to the polyscope mesh,
        // we'll use the body's visual properties to indicate hits

        // Store original color if we haven't already
        if (!body->visualProperties.count("contactHitsTracked")) {
            if (body->visualProperties.count("colorR")) {
                body->visualProperties["origColorR"] = body->visualProperties["colorR"];
                body->visualProperties["origColorG"] = body->visualProperties["colorG"];
                body->visualProperties["origColorB"] = body->visualProperties["colorB"];
            } else {
                body->visualProperties["origColorR"] = 0.5f;
                body->visualProperties["origColorG"] = 0.5f;
                body->visualProperties["origColorB"] = 0.5f;
            }
            body->visualProperties["contactHitsTracked"] = 1.0f;
        }

        // Set color based on hit intensity
        float intensity = std::min(1.0f, static_cast<float>(hitData.maxHits) / s_visParams.hitCountThreshold);

        // Gradient from original color to red based on hit intensity
        float origR = body->visualProperties["origColorR"];
        float origG = body->visualProperties["origColorG"];
        float origB = body->visualProperties["origColorB"];

        // Blend toward hit color for more hits
        if (s_visParams.blendWithOriginalColor) {
            body->visualProperties["colorR"] = origR + intensity * (s_visParams.hitColor.x() - origR);
            body->visualProperties["colorG"] = origG + intensity * (s_visParams.hitColor.y() - origG);
            body->visualProperties["colorB"] = origB + intensity * (s_visParams.hitColor.z() - origB);
        } else {
            body->visualProperties["colorR"] = s_visParams.hitColor.x() * intensity;
            body->visualProperties["colorG"] = s_visParams.hitColor.y() * intensity;
            body->visualProperties["colorB"] = s_visParams.hitColor.z() * intensity;
        }

        // Apply the visual changes
        body->applyVisualProperties();

        // Print hit count information (for debugging)
        if (hitData.maxHits > 0 && s_visParams.showMaxHitLabels) {
            std::cout << "Body " << bodyId << " max hits: " << hitData.maxHits << std::endl;
        }
    }

    // Handle tangent visualization if enabled
    if (s_tangentVisualizationEnabled) {
        for (auto& [bodyId, hitData] : s_bodyFaceHits) {
            // Clean up previous tangent visualization
            std::string pcName = "body_" + std::to_string(bodyId) + "_tangents";
            polyscope::removePointCloud(pcName, false);

            // Find the corresponding rigid body
            RigidBody* body = nullptr;
            for (auto* b : bodies) {
                if (b->id == bodyId) {
                    body = b;
                    break;
                }
            }

            if (!body || !body->mesh || hitData.maxHits == 0) continue;

            // Collect points and tangent vectors for visualization
            std::vector<Eigen::Vector3f> points;
            std::vector<Eigen::Vector3f> tangents;

            // For each face with hits, add a point and tangent
            for (size_t i = 0; i < hitData.hitCounts.size(); i++) {
                if (hitData.hitCounts[i] > 0 && hitData.tangentDirections[i].norm() > 1e-6f) {
                    // Calculate an approximation of the face center
                    // For now, use a simple offset from body center based on the face index
                    Eigen::Vector3f faceDir = Eigen::Vector3f(
                        std::cos(static_cast<float>(i) * 0.5f),
                        std::sin(static_cast<float>(i) * 0.5f),
                        std::cos(static_cast<float>(i) * 0.3f)
                    ).normalized();

                    // Place the visualization point on the surface of the body
                    // Using an approximate radius based on geometry type
                    float approxRadius = 0.5f; // Default fallback value

                    if (body->geometry) {
                        // Get appropriate radius based on geometry type
                        switch (body->geometry->getType()) {
                            case kSphere:
                                // For spheres, directly use the radius
                                approxRadius = dynamic_cast<Sphere*>(body->geometry.get())->radius;
                                break;

                            case kBox:
                                {
                                    // For boxes, use average half-dimension
                                    Box* box = dynamic_cast<Box*>(body->geometry.get());
                                    approxRadius = (box->dim.x() + box->dim.y() + box->dim.z()) / 6.0f;
                                }
                                break;

                            case kCylinder:
                                {
                                    // For cylinders, use the radius
                                    Cylinder* cyl = dynamic_cast<Cylinder*>(body->geometry.get());
                                    approxRadius = cyl->radius;
                                }
                                break;

                            case kPlane:
                                // Planes don't have a meaningful radius
                                approxRadius = 0.5f;
                                break;

                            default:
                                // Use default radius
                                approxRadius = 0.5f;
                                break;
                        }
                    }

                    // Cap the radius to reasonable values
                    approxRadius = std::min(std::max(approxRadius, 0.1f), 2.0f);

                    Eigen::Vector3f faceCenter = body->x + faceDir * approxRadius;

                    // Transform the tangent to world space
                    Eigen::Vector3f worldTangent = body->q * hitData.tangentDirections[i];

                    // Scale the tangent by hit count for visual emphasis
                    float hitScale = s_visParams.vectorScale * std::min(5.0f, static_cast<float>(hitData.hitCounts[i]));
                    worldTangent *= hitScale;

                    points.push_back(faceCenter);
                    tangents.push_back(worldTangent);
                }
            }

            // Visualize the tangent vectors if we have any
            if (!points.empty()) {
                // Convert to format for Polyscope
                Eigen::MatrixXf P(points.size(), 3);
                Eigen::MatrixXf T(tangents.size(), 3);

                for (size_t i = 0; i < points.size(); i++) {
                    P.row(i) << points[i].x(), points[i].y(), points[i].z();
                    T.row(i) << tangents[i].x(), tangents[i].y(), tangents[i].z();
                }

                // Create point cloud visualization with vector field
                auto pc = polyscope::registerPointCloud(pcName, P);
                pc->setPointColor({s_visParams.pointColor.x(), s_visParams.pointColor.y(), s_visParams.pointColor.z()});
                pc->setPointRadius(s_visParams.pointRadius);
                pc->addVectorQuantity("tangent_directions", T)
                  ->setVectorColor({s_visParams.tangentColor.x(), s_visParams.tangentColor.y(), s_visParams.tangentColor.z()})
                  ->setVectorLengthScale(s_visParams.visualScale)
                  ->setEnabled(true);
            }
        }
    } else {
        // Clean up tangent visualizations if disabled
        for (auto& [bodyId, _] : s_bodyFaceHits) {
            std::string pcName = "body_" + std::to_string(bodyId) + "_tangents";
            polyscope::removePointCloud(pcName, false);
        }
    }
}