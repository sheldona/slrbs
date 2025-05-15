#pragma once

#include "util/Types.h"
#include "collision/BVH.h"
#include "collision/AABB.h"
#include <memory>
#include <vector>
#include <string>
#include <contact/FaceContactTracker.h>

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
    void drawContactVisualizationUI();
    // Pre-step callback with geometric stiffness support
    void preStep(RigidBodySystem& system, float h);

    void showAllMeshBVHs();
    void showAllMeshAABBs();

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
    float m_constraintErr;              // Total constraint error
    bool m_showContactHits = true;
    bool m_showContactTangents = true;  // Added missing variable for tangent visualization

    // Toggle flags for visualization
    bool m_showMeshBVH   = false;
    bool m_showMeshAABB  = false;

    // BVH and AABB helpers
    BVH  m_meshBVH;
    AABB m_sceneAABB;

    bool m_showAllBVHs  = false;
    bool m_showAllAABBs = false;
    bool m_enableSolverOpenMP = true;
    bool m_enableOpenMP = true;

    float m_newtonTolerance = 1e-6f;
    int   m_newtonMaxIter = 5;
    float m_newtonDamping = 0.98f;
    bool  m_loggingEnabled = false;

    // Proximal solver parameters
    float m_proximalAbsTol = 1e-5f;
    float m_proximalRelTol = 1e-5f;
    bool m_proximalExportEnabled = false;
    std::string m_proximalExportPath = "proximal_data";

    // FaceTracker logging
    bool m_faceTrackerLogging = false;
    std::string m_faceTrackerLogPath = "contact_logs";

    float m_bppStabilization = 250.0f;
    float m_bppPivotTolerance = 1e-5f;
    int m_bppMaxIterations = 100;

    // BoxPGS parameters
    float m_pgsStabilizationFactor = 0.3f;

    // Conjugate method parameters
    float m_conjTolerance = 1e-6f;
    int m_conjRestartInterval = 10;

    // PGSSM parameters
    int m_pgssmSubIter = 3;
    float m_pgssmGamma = 0.3f;

    float m_contactFriction = 0.8f;
    float m_contactRestitutionThreshold = 0.5f;
    float m_contactBaumgarte = 0.2f;
    float m_contactSlop = 0.01f;

    // FaceContactTracker parameters
    bool m_contactTrackerEnabled = true;
    bool m_contactTangentVisualizationEnabled = true;

    // Contact parameters
    bool m_contactWarmStarting = true;
    float m_contactWarmStartFactor = 0.8f;
    float m_contactMaxPenetration = 0.1f;
    float m_contactFrictionTangentScale = 1.0f;
    bool m_contactEnhancedFriction = false;
    int m_contactModelIndex = 1; // 0: COULOMB, 1: BOX, 2: CONE

    // Contact visualization/tracking parameters
    bool m_contactTracking = true;
    bool m_contactTangentVisualization = true;
    int m_contactVisModeIndex = 1; // 0: NONE, 1: COLOR_GRADIENT, 2: HEAT_MAP, 3: CUSTOM
    int m_contactTangentVisModeIndex = 1; // 0: NONE, 1: ARROWS, 2: STREAMLINES, 3: POINTS
    int m_contactHitThreshold = 50;
    bool m_contactHitDecay = false;
    float m_contactHitDecayRate = 0.05f;
    bool m_contactLogging = false;
    char m_contactLogPath[256] = "contact_logs";
    bool m_contactLogOptions[5] = {true, false, false, false, false};

    FaceContactTracker::VisualizationMode m_contactVisMode;
    FaceContactTracker::TangentVisualizationMode m_tangentVisMode;
    float m_hitThreshold;
    float m_visualizationScale;
    float m_pointRadius;
    float m_vectorScale;
    bool m_blendWithOriginalColor;
    bool m_showMaxHitLabels;
    bool m_enableHitDecay;
    float m_hitDecayRate;
};