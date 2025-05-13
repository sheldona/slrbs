#pragma once

#include "util/Types.h"
#include <memory>
#include <vector>
#include <string>

namespace polyscope {
    class SurfaceMesh;
    class PointCloud;
    class CameraParameters;
}

class Contact;
class RigidBodySystem;
class RigidBody;
class RigidBodySystemState;
class Joint;

class SimViewer {
public:
    SimViewer();
    virtual ~SimViewer();

    void start();
    void reset();
    void save();

private:
    // Built-in scenario creation methods
    void createMarbleBox();
    void createSphereOnBox();
    void createSwingingBox();
    void createCylinderOnPlane();
    void createCarScene();
    void createStack();

    // Additional scenarios
    void createRopeBridge();
    void createSphereSphereDistance();
    void createSphereInsideBox();
    void createBoxOnPlane();
    void createCylinderSphereTest();
    void createRopeLadder();

    // Custom scenarios
    void createCustomScenario1();
    void createCustomScenario2();
    void createCustomScenario3();
    void createCustomScenario4();
    void createCustomScenario5();
    void createCustomScenario6();
    void createCustomScenario7();
    void createCustomScenario8();
    void createCustomScenario9();

    // JSON scenario handling methods
    void loadScenarioFromJSON(const std::string& filename);
    void refreshScenariosList();
    void drawScenarioSelectionGUI();

    // Main rendering and GUI methods
    void draw();
    void drawGUI();
    void drawColorDebugUI();
    // Pre-step callback with geometric stiffness support
    void preStep(RigidBodySystem& system, float h);

private:
    // Simulation parameters
    float m_dt;                         // Time step parameter
    int m_subSteps;                     // Number of substeps per frame
    bool m_paused;                      // Pause the simulation
    bool m_stepOnce;                    // Advance the simulation by one frame and then stop
    bool m_enableCollisions;            // Enable/disable collisions
    bool m_enableScreenshots;           // Enable/disable saving screenshots
    bool m_drawContacts;                // Enable drawing contacts
    bool m_drawConstraints;             // Enable constraint visualization
    float m_dynamicsTime;               // Compute time for the dynamics step (in ms)
    std::unique_ptr<RigidBodySystemState> m_resetState;

    // Adaptive timestep and geometric stiffness damping
    bool m_adaptiveTimesteps;           // Toggle adaptive sub-stepping
    bool m_gsDamping;                   // Toggle geometric stiffness damping
    float m_alpha;                      // Control parameter for adaptive timesteps

    // UI state
    int m_selectedScenario;             // Currently selected scenario in the UI
    int m_selectedBodyIndex;            // Currently selected body in the hierarchy view
    std::vector<std::string> m_availableScenarios; // List of available JSON scenarios
    bool m_showUV = false;
    int m_frameCounter;                 // Frame number
    float m_kineticEnergy;              // System kinetic energy
    float m_constraintErr;               // Total constraint error
    bool m_showContactHits = true;
};