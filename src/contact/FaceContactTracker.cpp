#include "contact/FaceContactTracker.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include <algorithm>
#include <iostream>

// Static member initialization
std::map<int, FaceContactTracker::FaceHitData> FaceContactTracker::s_bodyFaceHits;
bool FaceContactTracker::s_visualizationEnabled = true;

void FaceContactTracker::initialize() {
    s_bodyFaceHits.clear();
    s_visualizationEnabled = true;
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

void FaceContactTracker::reset() {
    for (auto& [bodyId, data] : s_bodyFaceHits) {
        data.reset();
    }
}

void FaceContactTracker::updateVisualization(RigidBodySystem* system) {
    if (!s_visualizationEnabled || !system) return;

    // Get bodies from the provided system
    auto& bodies = system->getBodies();

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
        float intensity = std::min(1.0f, hitData.maxHits / 50.0f); // Cap at 50 hits

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
}