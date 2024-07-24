#pragma once

/**
 * @file SimViewer.h
 *
 * @brief Viewer for a cloth simulation application.
 *
 */

#include "util/Types.h"
#include <memory>
#include <vector>

namespace polyscope
{
    class SurfaceMesh;
    class PointCloud;
}

class Contact;
class RigidBodySystem;
class RigidBody;
class RigidBodyState;
class RigidBodySystemState;


class SimViewer 
{
public:
    SimViewer();
    virtual ~SimViewer();

    void start();
    void reset();
    void save();

private:
    void createStack();
    void createMarbleBox();
    void createSphereOnBox();
    void createSwingingBox();
    void createCylinderOnPlane();
    void createCarScene();
    void createBridgeScene();

    void draw();
    void drawGUI();

    void preStep(RigidBodySystem& _rigidBodySystem, float h);

private:

    // Simulation parameters
    bool m_adaptiveTimesteps;
    bool m_gsDamping;
    float m_alpha;
    float m_dt;                         // Time step parameter.
    int m_subSteps;
    bool m_paused;                      // Pause the simulation.
    bool m_stepOnce;                    // Advance the simulation by one frame and then stop.
    bool m_enableCollisions;            // enable/disable collisions
    bool m_enableScreenshots;           // enable/disable saving screenshots
    bool m_enableLogging;
    bool m_drawContacts;                // enable drawing contacts
    bool m_drawConstraints;             // enable constraint viz
    float m_dynamicsTime;               // Compute time for the dynamics step (in ms)
    int m_frameCounter;                 // Frame number
    std::unique_ptr<RigidBodySystemState> m_resetState;

};
