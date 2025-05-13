#include "viewer/SimViewer.h"

#include "polyscope/polyscope.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/view.h"
#include <functional>
#include "imgui.h"

#include <chrono>
#include <iostream>
#include <functional>

#include "contact/Contact.h"
#include "contact/FaceContactTracker.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBodyState.h"
#include "rigidbody/Scenarios.h"
#include "util/ScenarioLoader.h"
#include "util/VisualProperties.h"

using namespace std;

// Static variables and helper functions for visualization and UI
namespace
{
    static RigidBodySystem* m_rigidBodySystem = new RigidBodySystem;

    static const char* strContacts    = "contacts";
    static const char* strJointPoints = "jointsPoint";
    static const char* strJointCurve  = "jointsCurve";

    static void updateRigidBodyMeshes(RigidBodySystem& _rigidBodySystem)
    {
        auto& bodies = _rigidBodySystem.getBodies();
        for (unsigned int k = 0; k < bodies.size(); ++k)
        {
            if (!bodies[k]->mesh) continue;

            Eigen::Isometry3f tm = Eigen::Isometry3f::Identity();
            tm.linear()      = bodies[k]->q.toRotationMatrix();
            tm.translation() = bodies[k]->x;
            bodies[k]->mesh->setTransform(glm::make_mat4x4(tm.data()));

            // Apply body‐specific
            bodies[k]->applyVisualProperties();

            // Fallback to global
            bool foundGlobal = false;
            for (auto const& prop : g_visualProperties.bodyProperties)
            {
                if (prop.bodyIndex == (int)k)
                {
                    bodies[k]->mesh->setSurfaceColor({prop.colorR, prop.colorG, prop.colorB});
                    bodies[k]->mesh->setTransparency(prop.transparency);
                    bodies[k]->mesh->setSmoothShade(prop.smoothShade);
                    bodies[k]->mesh->setEdgeWidth(prop.edgeWidth);
                    foundGlobal = true;
                    break;
                }
            }

            // Default
            if (!foundGlobal && bodies[k]->visualProperties.empty())
            {
                if (bodies[k]->fixed)
                {
                    bodies[k]->mesh->setSurfaceColor({0.5f, 0.5f, 0.5f});
                    bodies[k]->mesh->setTransparency(1.0f);
                    bodies[k]->mesh->setSmoothShade(false);
                }
                else
                {
                    float hue = fmodf(k * 0.618033988749895f, 1.0f);
                    float h   = hue * 6.0f;
                    float c   = 0.9f;
                    float x   = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
                    float r = 0, g = 0, b = 0;
                    if      (h < 1.0f) { r = c; g = x; }
                    else if (h < 2.0f) { r = x; g = c; }
                    else if (h < 3.0f) { g = c; b = x; }
                    else if (h < 4.0f) { g = x; b = c; }
                    else if (h < 5.0f) { r = x; b = c; }
                    else               { r = c; b = x; }
                    bodies[k]->mesh->setSurfaceColor({r, g, b});
                    bodies[k]->mesh->setTransparency(1.0f);
                    bodies[k]->mesh->setSmoothShade(true);
                }
            }
        }
    }

    static float computeKineticEnergy(RigidBodySystem& _rigidBodySystem)
    {
        float kinEnergy = 0.0f;
        for (RigidBody* b : _rigidBodySystem.getBodies())
        {
            const auto I = b->q * b->Ibody * b->q.inverse();
            kinEnergy += b->mass * b->xdot.squaredNorm();
            kinEnergy += b->omega.dot(I * b->omega);
        }

        return 0.5f*kinEnergy;
    }

    static void updateContactPoints(RigidBodySystem& _rigidBodySystem)
    {
        const auto& contacts    = _rigidBodySystem.getContacts();
        const unsigned int nCtrs = (unsigned int)contacts.size();

        if (nCtrs == 0)
        {
            polyscope::removePointCloud(strContacts);
        }
        else
        {
            Eigen::MatrixXf P(nCtrs, 3), N(nCtrs, 3);
            for (unsigned int i = 0; i < nCtrs; ++i)
            {
                P.row(i) << contacts[i]->p(0), contacts[i]->p(1), contacts[i]->p(2);
                N.row(i) << contacts[i]->n(0), contacts[i]->n(1), contacts[i]->n(2);
            }
            auto pc = polyscope::registerPointCloud(strContacts, P);
            pc->setPointColor({1,0,0});
            pc->setPointRadius(0.005f);
            pc->addVectorQuantity("normal", N)
              ->setVectorColor({1,1,0})
              ->setVectorLengthScale(0.05f)
              ->setEnabled(true);
        }
    }

    static void updateJointViz(RigidBodySystem& _rigidBodySystem)
    {
        const auto& joints     = _rigidBodySystem.getJoints();
        const unsigned int nJn = (unsigned int)joints.size();

        if (nJn == 0)
        {
            polyscope::removePointCloud(strJointPoints);
            polyscope::removeCurveNetwork(strJointCurve);
        }
        else
        {
            Eigen::MatrixXf P(2*nJn, 3);
            Eigen::MatrixXi E(nJn, 2);
            for (unsigned int i = 0; i < nJn; ++i)
            {
                auto p0 = joints[i]->body0->q * joints[i]->r0 + joints[i]->body0->x;
                auto p1 = joints[i]->body1->q * joints[i]->r1 + joints[i]->body1->x;
                P.row(2*i  ) = p0;
                P.row(2*i+1) = p1;
                E.row(i)     = Eigen::Vector2i(2*i, 2*i+1);
            }
            auto pc = polyscope::registerPointCloud(strJointPoints, P);
            pc->setPointColor({0,0,1});
            pc->setPointRadius(0.005f);
            polyscope::registerCurveNetwork(strJointCurve, P, E)
                     ->setRadius(0.002f);
        }
    }
}

SimViewer::SimViewer()
    : m_adaptiveTimesteps(false),
      m_gsDamping(false),
      m_alpha(0.01f),
      m_dt(0.01667f),
      m_subSteps(1),
      m_dynamicsTime(0.0f),
      m_frameCounter(0),
      m_kineticEnergy(0.0f),
      m_constraintErr(0.0f),
      m_paused(true),
      m_stepOnce(false),
      m_enableCollisions(true),
      m_enableScreenshots(false),
      m_drawContacts(true),
      m_drawConstraints(true),
      m_selectedScenario(-1),
      m_selectedBodyIndex(-1),
      m_showContactHits(true)
{
    m_resetState = make_unique<RigidBodySystemState>(*m_rigidBodySystem);
    refreshScenariosList();
    reset();
    FaceContactTracker::initialize();
}

SimViewer::~SimViewer() = default;

void SimViewer::reset()
{
    cout << "---- Reset ----" << endl;
    m_resetState->restore(*m_rigidBodySystem);
    m_dynamicsTime = 0.0f;
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();

    // Reset face contact tracking
    FaceContactTracker::reset();

    // Initialize tracking for all bodies
    for (auto* body : m_rigidBodySystem->getBodies()) {
        FaceContactTracker::initializeForBody(body);
    }
}

void SimViewer::save()
{
    cout << "---- Saving current state ----" << endl;
    m_resetState->save(*m_rigidBodySystem);
}

void SimViewer::start()
{
    // Polyscope options
    polyscope::options::programName = "slrbs";
    polyscope::options::verbosity   = 0;
    polyscope::options::usePrefsFile = false;
    polyscope::options::alwaysRedraw = true;
    polyscope::options::ssaaFactor   = 2;
    polyscope::options::openImGuiWindowForUserCallback = true;
    polyscope::options::groundPlaneHeightFactor = 0.0f;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::TileReflection;
    polyscope::options::buildGui       = false;
    polyscope::options::maxFPS         = -1;
    polyscope::options::groundPlaneEnabled = true;
    polyscope::options::screenshotExtension = ".png";

    polyscope::init();

    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::lengthScale = 10.0f;
    polyscope::state::boundingBox =
      std::tuple<glm::vec3, glm::vec3>{{-5,0,-5}, {5,5,5}};

    polyscope::state::userCallback =
      std::bind(&SimViewer::draw, this);

    m_rigidBodySystem->setPreStepFunc(
    [this](RigidBodySystem& system, float h) {
        this->preStep(system, h);
    });

    polyscope::show();
}

void SimViewer::refreshScenariosList()
{
    m_availableScenarios = ScenarioLoader::listAvailableScenarios();
    if (m_availableScenarios.empty())
        cout << "No JSON scenario files found." << endl;
    else
        cout << "Found " << m_availableScenarios.size() << " JSON scenario files." << endl;
    m_selectedScenario = -1;
}

void SimViewer::loadScenarioFromJSON(const string& filename)
{
    polyscope::removeAllStructures();
    if (ScenarioLoader::loadFromFile(*m_rigidBodySystem, filename))
    {
        cout << "Successfully loaded scenario from " << filename << endl;
        updateRigidBodyMeshes(*m_rigidBodySystem);
        m_resetState->save(*m_rigidBodySystem);
        polyscope::resetScreenshotIndex();
    }
    else
    {
        cerr << "Failed to load scenario from " << filename << endl;
    }
}

void SimViewer::drawScenarioSelectionGUI()
{
    if (!ImGui::CollapsingHeader("JSON Scenarios", ImGuiTreeNodeFlags_DefaultOpen))
        return;
    if (ImGui::Button("Refresh Scenarios List"))
        refreshScenariosList();
    if (m_availableScenarios.empty())
    {
        ImGui::Text("No JSON scenario files found.");
    }
    else
    {
        ImGui::Text("Available Scenarios:");
        ImGui::Indent();
        for (int i = 0; i < (int)m_availableScenarios.size(); ++i)
        {
            if (ImGui::Selectable(m_availableScenarios[i].c_str(), m_selectedScenario == i))
                m_selectedScenario = i;
        }
        ImGui::Unindent();
        if (m_selectedScenario >= 0 && m_selectedScenario < (int)m_availableScenarios.size())
        {
            if (ImGui::Button("Load Selected Scenario"))
                loadScenarioFromJSON(m_availableScenarios[m_selectedScenario]);
        }
    }
}

void SimViewer::drawGUI()
{
    // Simulation Controls
    ImGui::Begin("Simulation Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::Text("Time: %.2f s", m_dynamicsTime);
        if (ImGui::Button(m_paused ? "Play" : "Pause", ImVec2(100,0)))
            m_paused = !m_paused;
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(100,0))) m_stepOnce = true;
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(100,0))) reset();
        ImGui::SameLine();
        if (ImGui::Button("Save", ImVec2(100,0))) save();
        ImGui::Separator();
        ImGui::Checkbox("Enable collision detection", &m_enableCollisions);
        if (m_enableCollisions != m_rigidBodySystem->getEnableCollisionDetection())
            m_rigidBodySystem->setEnableCollisionDetection(m_enableCollisions);
        ImGui::Checkbox("Draw contacts", &m_drawContacts);
        ImGui::Checkbox("Draw constraints", &m_drawConstraints);
        ImGui::Checkbox("Enable screenshots", &m_enableScreenshots);
        ImGui::Checkbox("Adaptive timesteps", &m_adaptiveTimesteps);
        ImGui::Checkbox("GS damping", &m_gsDamping);
        ImGui::Checkbox("Show contact hits", &m_showContactHits);
        FaceContactTracker::setVisualizationEnabled(m_showContactHits);
    }
    ImGui::End();

    ImGui::Begin("Solver Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PushItemWidth(150);
    {
        // Time step & substeps
        ImGui::SliderFloat("Time step", &m_dt, 0.001f, 0.1f, "%.3f s");
        ImGui::SliderInt("Sub-steps", &m_subSteps, 1, 20);

        // Solver iterations & friction
        int iters = m_rigidBodySystem->getSolverIterations();
        if (ImGui::SliderInt("Solver iterations", &iters, 1, 100))
            m_rigidBodySystem->setSolverIterations(iters);
        ImGui::SliderFloat("Friction coeff.", &Contact::mu, 0.0f, 2.0f, "%.2f");

        // Integrator selection with tooltip
        static const char* integrators[] = {
            "Explicit Euler", "Symplectic Euler", "Verlet", "RK4", "Implicit Euler", "Newton"
        };
        int currentIntegrator = static_cast<int>(m_rigidBodySystem->getIntegratorType());
        if (ImGui::Combo("Integrator", &currentIntegrator, integrators, IM_ARRAYSIZE(integrators))) {
            m_rigidBodySystem->setIntegratorType(static_cast<IntegratorType>(currentIntegrator));
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Choose your integration method. "
                              "Implicit Euler offers extra stability options.");

        // If using Implicit Euler, show extra parameters
        if (m_rigidBodySystem->getIntegrationMethod() == IntegrationMethod::IMPLICIT_EULER) {
            ImGui::Separator();
            ImGui::Text("Implicit-Euler Settings");

            float d = m_rigidBodySystem->getImplicitDamping();
            if (ImGui::SliderFloat("Damping", &d, 0.0f, 1.0f, "%.2f"))
                m_rigidBodySystem->setImplicitDamping(d);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Global velocity damping factor.");

            float gd = m_rigidBodySystem->getGyroscopicDamping();
            if (ImGui::SliderFloat("Gyro Damping", &gd, 0.0f, 1.0f, "%.2f"))
                m_rigidBodySystem->setGyroscopicDamping(gd);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Damping for gyroscopic forces.");

            bool lim = m_rigidBodySystem->getVelocityLimitingEnabled();
            if (ImGui::Checkbox("Limit Velocities", &lim))
                m_rigidBodySystem->setVelocityLimitingEnabled(lim);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle to clamp extreme velocities.");

            float mv = m_rigidBodySystem->getMaxLinearVelocity();
            if (ImGui::DragFloat("Max Lin Vel", &mv, 1.0f, 0.0f, 100000.0f))
                m_rigidBodySystem->setMaxLinearVelocity(mv);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Max linear speed clamp.");

            float mav = m_rigidBodySystem->getMaxAngularVelocity();
            if (ImGui::DragFloat("Max Ang Vel", &mav, 1.0f, 0.0f, 100000.0f))
                m_rigidBodySystem->setMaxAngularVelocity(mav);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Max angular speed clamp.");
        }

        ImGui::Separator();
        ImGui::Text("Solver Type:");
        SolverType st = m_rigidBodySystem->getSolverType();
        if (ImGui::RadioButton("BPP", st==SolverType::BPP)) m_rigidBodySystem->setSolverType(SolverType::BPP);
        if (ImGui::RadioButton("PGS", st==SolverType::PGS))           m_rigidBodySystem->setSolverType(SolverType::PGS);
        ImGui::SameLine();
        if (ImGui::RadioButton("PGSSM", st==SolverType::PGSSM))       m_rigidBodySystem->setSolverType(SolverType::PGSSM);
        if (ImGui::RadioButton("Conj Grad", st==SolverType::CONJ_GRADIENT)) m_rigidBodySystem->setSolverType(SolverType::CONJ_GRADIENT);
        ImGui::SameLine();
        if (ImGui::RadioButton("Conj Residual", st==SolverType::CONJ_RESIDUAL)) m_rigidBodySystem->setSolverType(SolverType::CONJ_RESIDUAL);
        if (ImGui::RadioButton("Proximal", st==SolverType::PROXIMAL)) m_rigidBodySystem->setSolverType(SolverType::PROXIMAL);
    }
    ImGui::PopItemWidth();
    ImGui::End();

    // Scenarios
    ImGui::Begin("Scenarios", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::CollapsingHeader("Built-in Scenarios", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // First row
        if (ImGui::Button("Sphere on box", ImVec2(150,0)))    createSphereOnBox();
        ImGui::SameLine();
        if (ImGui::Button("Marble box",    ImVec2(150,0)))    createMarbleBox();

        // Second row
        if (ImGui::Button("Swinging box",  ImVec2(150,0)))    createSwingingBox();
        ImGui::SameLine();
        if (ImGui::Button("Cylinder on plane", ImVec2(150,0))) createCylinderOnPlane();

        // Third row
        if (ImGui::Button("Car scene",     ImVec2(150,0)))    createCarScene();
        ImGui::SameLine();
        if (ImGui::Button("Stack",         ImVec2(150,0)))    createStack();

        // Fourth row - Additional scenarios
        if (ImGui::Button("Rope Bridge",   ImVec2(150,0)))    createRopeBridge();
        ImGui::SameLine();
        if (ImGui::Button("Sphere-Sphere", ImVec2(150,0)))    createSphereSphereDistance();

        // Fifth row
        if (ImGui::Button("Sphere in Box", ImVec2(150,0)))    createSphereInsideBox();
        ImGui::SameLine();
        if (ImGui::Button("Box on Plane",  ImVec2(150,0)))    createBoxOnPlane();

        // Sixth row
        if (ImGui::Button("Cylinder-Sphere", ImVec2(150,0)))  createCylinderSphereTest();
        ImGui::SameLine();
        if (ImGui::Button("Rope Ladder",   ImVec2(150,0)))    createRopeLadder();
    }

    // Custom scenarios
    if (ImGui::CollapsingHeader("Custom Scenarios"))
    {
        // First row
        if (ImGui::Button("Double Pendulum", ImVec2(150,0)))  createCustomScenario1();
        ImGui::SameLine();
        if (ImGui::Button("Spherical Joint", ImVec2(150,0)))  createCustomScenario2();

        // Second row
        if (ImGui::Button("2 Prismatics", ImVec2(150,0)))     createCustomScenario3();
        ImGui::SameLine();
        if (ImGui::Button("Box-Sphere Joint", ImVec2(150,0))) createCustomScenario4();

        // Third row
        if (ImGui::Button("3 Prismatics", ImVec2(150,0)))     createCustomScenario5();
        ImGui::SameLine();
        if (ImGui::Button("Hinge Joint", ImVec2(150,0)))      createCustomScenario6();

        // Fourth row
        if (ImGui::Button("Tensile Table", ImVec2(150,0)))    createCustomScenario7();
        ImGui::SameLine();
        if (ImGui::Button("Spheres in Box", ImVec2(150,0)))   createCustomScenario8();

        // Fifth row
        if (ImGui::Button("Box with Faces", ImVec2(150,0)))   createCustomScenario9();
    }

    // JSON Scenarios section
    drawScenarioSelectionGUI();
    ImGui::End();

    // Scene Hierarchy
    ImGui::Begin("Scene Hierarchy", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        auto& bodies = m_rigidBodySystem->getBodies();
        if (ImGui::TreeNode("Rigid Bodies"))
        {
            for (size_t i = 0; i < bodies.size(); ++i)
            {
                char buf[64];
                string name = bodies[i]->mesh ? bodies[i]->mesh->name : "<no mesh>";
                snprintf(buf,64,"Body %zu: %s", i, name.c_str());
                bool sel = (m_selectedBodyIndex==(int)i);
                if (ImGui::Selectable(buf, sel)) m_selectedBodyIndex = i;
                if (ImGui::BeginPopupContextItem())
                {
                    if (ImGui::MenuItem("Focus Camera"))
                    {
                        if (bodies[i]->mesh)
                        {
                            polyscope::view::lookAt(
                                glm::vec3(bodies[i]->x[0], bodies[i]->x[1], bodies[i]->x[2]),
                                glm::vec3(0,0,0), false
                            );
                        }
                    }
                    if (ImGui::MenuItem("Toggle Fixed"))
                        bodies[i]->fixed = !bodies[i]->fixed;
                    ImGui::EndPopup();
                }
            }
            ImGui::TreePop();
        }
        const auto& joints = m_rigidBodySystem->getJoints();
        if (!joints.empty() && ImGui::TreeNode("Joints"))
        {
            for (size_t i = 0; i < joints.size(); ++i)
            {
                char buf[64];
                snprintf(buf,64,"Joint %zu: %s", i, joints[i]->getTypeName().c_str());
                if (ImGui::TreeNode(buf))
                {
                    ImGui::Text("Body 0: %d", joints[i]->body0->id);
                    ImGui::Text("Body 1: %d", joints[i]->body1->id);
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }
        if (m_selectedBodyIndex >= 0 && m_selectedBodyIndex < (int)bodies.size())
        {
            if (ImGui::TreeNode("Selected Body Properties"))
            {
                auto* body = bodies[m_selectedBodyIndex];
                ImGui::Checkbox("Fixed", &body->fixed);
                ImGui::DragFloat3("Position", body->x.data(), 0.1f);
                float quat[4] = {body->q.w(), body->q.x(), body->q.y(), body->q.z()};
                if (ImGui::DragFloat4("Orientation (WXYZ)", quat, 0.01f))
                {
                    body->q = Eigen::Quaternionf(quat[0], quat[1], quat[2], quat[3]);
                    body->q.normalize();
                }
                ImGui::DragFloat3("Linear Velocity",  body->xdot.data(), 0.1f);
                ImGui::DragFloat3("Angular Velocity", body->omega.data(), 0.1f);

                if (ImGui::TreeNode("Material Properties"))
                {
                    float r  = body->restitution;
                    float f  = body->friction;
                    float d  = body->density;
                    if (ImGui::SliderFloat("Restitution", &r, 0,1)) body->restitution = r;
                    if (ImGui::SliderFloat("Friction",    &f, 0,1)) body->friction    = f;
                    if (ImGui::DragFloat("Density", &d, 0.1f, 0.1f, 10000.0f)) body->density = d;
                    ImGui::TreePop();
                }

                if (body->mesh && ImGui::TreeNode("Visual Properties"))
                {
                    float col[3] = {0.5f,0.5f,0.5f};
                    if (body->visualProperties.count("colorR"))
                    {
                        col[0] = body->visualProperties["colorR"];
                        col[1] = body->visualProperties["colorG"];
                        col[2] = body->visualProperties["colorB"];
                    }
                    if (ImGui::ColorEdit3("Color", col))
                    {
                        body->visualProperties["colorR"] = col[0];
                        body->visualProperties["colorG"] = col[1];
                        body->visualProperties["colorB"] = col[2];
                        body->applyVisualProperties();
                    }
                    float trans = body->visualProperties.count("transparency")
                                  ? body->visualProperties["transparency"]
                                  : 1.0f;
                    if (ImGui::SliderFloat("Transparency", &trans, 0,1))
                    {
                        body->visualProperties["transparency"] = trans;
                        body->applyVisualProperties();
                    }
                    bool ss = body->visualProperties.count("smoothShade")
                              ? (body->visualProperties["smoothShade"]>0.5f)
                              : false;
                    if (ImGui::Checkbox("Smooth Shading", &ss))
                    {
                        body->visualProperties["smoothShade"] = ss?1.0f:0.0f;
                        body->applyVisualProperties();
                    }
                    float ew = body->visualProperties.count("edgeWidth")
                               ? body->visualProperties["edgeWidth"]
                               : 1.0f;
                    if (ImGui::SliderFloat("Edge Width", &ew, 0.0f,5.0f))
                    {
                        body->visualProperties["edgeWidth"] = ew;
                        body->applyVisualProperties();
                    }
                    ImGui::TreePop();
                }

                ImGui::TreePop();
            }
        }
    }
    ImGui::End();

    // Debug window
    ImGui::Begin("Debug", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        drawColorDebugUI();
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Polyscope Debug"))
        {
            // Ground plane toggle
            bool gp = polyscope::options::groundPlaneEnabled;
            if (ImGui::Checkbox("Ground Plane", &gp))
                polyscope::options::groundPlaneEnabled = gp;

            // Ground plane mode
            static const char* modes[] = {"None","Tile","TileReflection","Shadow"};
            int mode = (int)polyscope::options::groundPlaneMode;
            if (ImGui::Combo("Ground Plane Mode", &mode, modes, IM_ARRAYSIZE(modes)))
                polyscope::options::groundPlaneMode =
                  (polyscope::GroundPlaneMode)mode;

            // Ground plane height (ScaledValue<float>)
            {
                auto& gph = polyscope::options::groundPlaneHeightFactor;
                float h = gph.asAbsolute();
                if (ImGui::SliderFloat("Ground Plane Height", &h, -2.0f, 2.0f)) {
                    gph = polyscope::ScaledValue<float>::absolute(h);
                }
            }

            // Rendering options
            ImGui::Separator();
            ImGui::Text("Rendering Options:");
            ImGui::SliderInt("SSAA Factor", &polyscope::options::ssaaFactor, 1, 4);

            // Camera controls
            ImGui::Separator();
            ImGui::Text("Camera Controls:");
            if (ImGui::Button("Top View"))
                polyscope::view::lookAt({0,10,0},{0,0,0},{0,0,1},false);
            ImGui::SameLine();
            if (ImGui::Button("Side View"))
                polyscope::view::lookAt({10,0,0},{0,0,0},false);
            ImGui::SameLine();
            if (ImGui::Button("Front View"))
                polyscope::view::lookAt({0,0,10},{0,0,0},false);
        }
    }
    ImGui::End();

    // Performance window
    ImGui::Begin("Performance", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::Text("Kinetic energy: %8.3f ", m_kineticEnergy);
        ImGui::Text("Constraint error: %6.6f ", m_constraintErr);
        ImGui::Text("Physics step time: %.3f ms", m_dynamicsTime);
        static float fpsVals[120]={};
        static int   fpsOff=0;
        static float fpsAcc=0, avgF=0;
        static auto  lastT=chrono::high_resolution_clock::now();

        auto curr = chrono::high_resolution_clock::now();
        float dt = chrono::duration<float>(curr-lastT).count();
        lastT = curr;
        fpsAcc += dt;
        float fps = 1.0f/dt;
        fpsVals[fpsOff] = fps;
        fpsOff = (fpsOff+1)%120;
        if (fpsAcc>0.5f)
        {
            fpsAcc = 0;
            float sum=0;
            for (float v:fpsVals) sum += v;
            avgF = sum/120;
        }
        ImGui::Text("Framerate: %.1f FPS", avgF);
        ImGui::PlotLines("FPS", fpsVals, 120, fpsOff, nullptr, 0.0f, 120.0f, ImVec2(200,50));

        ImGui::Separator();
        ImGui::Text("Bodies:   %zu", m_rigidBodySystem->getBodies().size());
        ImGui::Text("Contacts: %zu", m_rigidBodySystem->getContacts().size());
        ImGui::Text("Joints:   %zu", m_rigidBodySystem->getJoints().size());
    }
    ImGui::End();
}

// Improved SimViewer::draw with refactored substepping and geometry stiffness handling
void SimViewer::draw() {
    drawGUI();
    if (m_paused && !m_stepOnce) return;
    auto t0 = chrono::high_resolution_clock::now();

    // Gather bodies, joints, contacts once
    auto& bodies   = m_rigidBodySystem->getBodies();
    auto& joints   = m_rigidBodySystem->getJoints();
    auto& contacts = m_rigidBodySystem->getContacts();

    // 1) Adaptive substepping based on geometric stiffness (CFL condition)
    if (m_adaptiveTimesteps) {
        // Zero accumulated stiffness
        for (auto* b : bodies) b->gsSum.setZero();
        // Accumulate stiffness from joints and contacts
        auto accumulateGs = [&](auto* element) {
            element->computeGeometricStiffness();
            element->body0->gsSum += element->G0;
            element->body1->gsSum += element->G1;
        };
        for (auto* j : joints)   accumulateGs(j);
        for (auto* c : contacts) accumulateGs(c);

        // Compute maximum stiffness-to-mass/inertia ratio
        float maxRatio = 0.0f;
        for (auto* b : bodies) {
            if (b->fixed) continue;
            // translational
            float massInv = 1.0f / b->mass;
            for (int i = 0; i < 3; ++i) {
                maxRatio = max(maxRatio, 2.0f * b->gsSum.col(i).norm() * massInv);
            }
            // rotational
            Eigen::Matrix3f Iworld = b->q * b->Ibody * b->q.inverse();
            for (int i = 0; i < 3; ++i) {
                maxRatio = max(maxRatio, 2.0f * b->gsSum.col(i+3).norm() / Iworld(i,i));
            }
        }
        if (maxRatio > 1e-6f) {
            float dtCFL = 2.0f * m_alpha * sqrtf(1.0f / maxRatio);
            m_subSteps = clamp((int)ceil(m_dt / dtCFL), 1, 1000);
        }
    }

    // 2) Integrate in sub-steps
    float dtSub = m_dt / float(m_subSteps);
    for (int i = 0; i < m_subSteps; ++i) {
        m_rigidBodySystem->step(dtSub);
    }

    // 3) Visualization update
    updateRigidBodyMeshes(*m_rigidBodySystem);
    if (m_showContactHits) {
        FaceContactTracker::updateVisualization(m_rigidBodySystem);
    }
    if (m_drawContacts) updateContactPoints(*m_rigidBodySystem);
    else polyscope::removePointCloud(strContacts);

    if (m_drawConstraints) updateJointViz(*m_rigidBodySystem);
    else {
        polyscope::removePointCloud(strJointPoints);
        polyscope::removeCurveNetwork(strJointCurve);
    }

    // 4) Record dynamics time and optionally screenshot
    auto t1 = chrono::high_resolution_clock::now();
    m_dynamicsTime = chrono::duration<float, milli>(t1 - t0).count();
    if (m_enableScreenshots) polyscope::screenshot(false);

    // 5) Update diagnostics
    m_kineticEnergy = computeKineticEnergy(*m_rigidBodySystem);
    m_constraintErr = 0.0f;
    for (auto* j : m_rigidBodySystem->getJoints()) {
        m_constraintErr += j->phi.lpNorm<1>();
    }

    m_stepOnce = false;
}

void SimViewer::createMarbleBox()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createMarbleBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSphereOnBox()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createSphereOnBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSwingingBox()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createSwingingBoxes(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCylinderOnPlane()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCylinderOnPlane(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCarScene()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCarScene(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createStack()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createStack(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createRopeBridge()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createRopeBridgeScene(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSphereSphereDistance()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createSphereSphereDistance(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSphereInsideBox()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createSphereInsideBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createBoxOnPlane()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createBoxOnPlane(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCylinderSphereTest()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Eigen::AngleAxisf aa(0.0f, Eigen::Vector3f(0, 0, 1)); // Default angle
    Scenarios::createCylinderSphereTest(*m_rigidBodySystem, aa);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createRopeLadder()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createRopeLadder(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

// Custom scenarios
void SimViewer::createCustomScenario1()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario2()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario2(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario3()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario3(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario4()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario4(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario5()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario5(*m_rigidBodySystem, 1, 1, 1);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario6()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario6(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario7()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario7(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario8()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario8(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCustomScenario9()
{
    polyscope::removeAllStructures();
    g_visualProperties.clear();
    for (auto* body : m_rigidBodySystem->getBodies()) {
        body->visualProperties.clear();
    }
    Scenarios::createCustomScenario9(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::preStep(RigidBodySystem& system, float h) {
    auto& bodies   = system.getBodies();
    auto& joints   = system.getJoints();
    auto& contacts = system.getContacts();

    // 1) Reset all geometric‐stiffness accumulators and damping
    for (auto* b : bodies) {
        b->clearGeometricStiffness();
        b->gsDamp.setZero();
    }

    // 2) Accumulate geometric stiffness from joints & contacts (if enabled)
    if (m_gsDamping) {
        auto accumulateGs = [&](auto* elm) {
            elm->computeGeometricStiffness();
            elm->body0->addGeometricStiffness(elm->G0);
            elm->body1->addGeometricStiffness(elm->G1);
        };
        for (auto* j : joints)   accumulateGs(j);
        for (auto* c : contacts) accumulateGs(c);
    }

    // 3) Adaptive timestep (CFL‐style) based on max stiffness‐to‐mass/inertia
    if (m_adaptiveTimesteps) {
        float maxRatio = 0.0f, minMass = FLT_MAX;
        for (auto* b : bodies) {
            if (b->fixed) continue;
            minMass = std::min(minMass, b->mass);
        }
        if (minMass < FLT_MAX) {
            for (auto* b : bodies) {
                if (b->fixed) continue;
                // translational modes
                float invM = 1.0f / b->mass;
                for (int i = 0; i < 3; ++i) {
                    maxRatio = std::max(maxRatio, 2.0f * b->gsSum.col(i).norm() * invM);
                }
                // rotational modes
                Eigen::Matrix3f Iw = b->q * b->Ibody * b->q.inverse();
                for (int i = 0; i < 3; ++i) {
                    maxRatio = std::max(maxRatio, 2.0f * b->gsSum.col(i+3).norm() / Iw(i,i));
                }
            }
            if (maxRatio > 1e-6f) {
                float dtCFL = 2.0f * m_alpha * std::sqrt(1.0f / maxRatio);
                // clamp substeps to avoid runaway
                m_subSteps = std::clamp((int)std::ceil(m_dt / dtCFL), 1, 1000);
            }
        }
    }

    // 4) Fold geometric‐stiffness into each body's inertia before the solver
    for (auto* b : bodies) {
        if (!b->fixed) {
            b->updateInertiaMatrixWithGs(h);
        }
    }
}


void SimViewer::drawColorDebugUI()
{
    if (ImGui::Button("Reset to Default"))
    {
        g_visualProperties.clear();
        for (auto* body : m_rigidBodySystem->getBodies()) {
            body->visualProperties.clear();
        }
        updateRigidBodyMeshes(*m_rigidBodySystem);
    }
}