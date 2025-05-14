#include "contact/FaceContactTracker.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "collision/Geometry.h"  // Add this to access geometry subclasses
#include <algorithm>
#include <iostream>

// Static member initialization
std::map<int, FaceContactTracker::FaceHitData> FaceContactTracker::s_bodyFaceHits;
bool FaceContactTracker::s_visualizationEnabled = true;
bool FaceContactTracker::s_tangentVisualizationEnabled = true;

void FaceContactTracker::initialize() {
    s_bodyFaceHits.clear();
    s_visualizationEnabled = true;
    s_tangentVisualizationEnabled = true;
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
    data.maxHits = 0;
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

void FaceContactTracker::reset() {
    for (auto& [bodyId, data] : s_bodyFaceHits) {
        data.reset();
    }
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
        float intensity = std::min(1.0f, static_cast<float>(hitData.maxHits) / 50.0f); // Cap at 50 hits

        // Gradient from original color to red based on hit intensity
        float origR = body->visualProperties["origColorR"];
        float origG = body->visualProperties["origColorG"];
        float origB = body->visualProperties["origColorB"];

        // Blend toward red for more hits
        body->visualProperties["colorR"] = origR + intensity * (1.0f - origR);
        body->visualProperties["colorG"] = origG * (1.0f - intensity);
        body->visualProperties["colorB"] = origB * (1.0f - intensity);

        // Apply the visual changes
        body->applyVisualProperties();

        // Print hit count information (for debugging)
        if (hitData.maxHits > 0) {
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
                    float hitScale = 0.01f * std::min(5.0f, static_cast<float>(hitData.hitCounts[i]));
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
                pc->setPointColor({0.8f, 0.2f, 0.2f}); // Red points
                pc->setPointRadius(0.005f);
                pc->addVectorQuantity("tangent_directions", T)
                  ->setVectorColor({0.0f, 0.8f, 0.2f}) // Green for tangents
                  ->setVectorLengthScale(1.0f)
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