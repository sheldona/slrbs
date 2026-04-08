#include "viewer/SimViewer.h"

#include "polyscope/polyscope.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/view.h"
#include "imgui.h"

#include <chrono>
#include <iostream>
#include <functional>

#include "contact/Contact.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBodyState.h"
#include "rigidbody/Scenarios.h"

using namespace std;

namespace
{
    static struct HydraulicStruct
    {
        HydraulicStruct() : 
            f_hyd(0.0f), rho(870.0f), x_spool(0.0f), 
            x_piston(0.0f), v_piston(0.0f),
            P_tank(1e5f), P_relief(20e6f), P_source(1e6f), 
            P_A(1e5f), P_B(1e5f), 
            R_piston(0.05f), R_rod(0.035f),
            beta(0.5e9f)
        {
        
        }

        void reset()
        {
            P_A = P_tank;
            P_B = P_tank;
            x_spool = 0.0f;
            x_piston = 0.0f;
            f_hyd = 0.0f;
            v_piston = 0.0;
        }

        float rho;          // hydraulic fluid density
        float x_spool;      // valve spool position (Typical range: -0.005 m to 0.0005m
        float v_piston;     // piston velocity (from rigid body)
        float x_piston;     // piston position (from rigid body)

        float R_piston;     // radius of the piston
        float R_rod;        // radius of the rod
        float V_cyl;        // current volume of cylinder
        float V_A, V_B;     // volumes in chamber A and B
        float P_A, P_B;     // pressures in chamber A and B
        float P_tank;       // tank pressure
        float P_relief;     // relief pressure
        float P_source;     // source pressure
        float f_hyd;
        float beta;         // bulk modulus of oil (Typical range: 0.7 GPa to 1.2 GPa)

    } hydraulicParams;

    static RigidBodySystem* m_rigidBodySystem = new RigidBodySystem;
    
    static const char* strContacts = "contacts";
    static const char* strJointPoints = "jointsPoint";
    static const char* strJointCurve = "jointsCurve";

    static void updateRigidBodyMeshes(RigidBodySystem& _rigidBodySystem)
    {
        auto& bodies = _rigidBodySystem.getBodies();
        for(unsigned int k = 0; k < bodies.size(); ++k)
        { 
            if (!bodies[k]->mesh) continue;

            Eigen::Isometry3f tm = Eigen::Isometry3f::Identity();
        
            // copy rotation part
            tm.linear() = bodies[k]->q.toRotationMatrix();

            // copy translation part
            tm.translation() = bodies[k]->x;

            bodies[k]->mesh->setTransform(glm::make_mat4x4(tm.data()));
        }
    }

    static void updateContactPoints(RigidBodySystem& _rigidBodySystem)
    {
        const auto& contacts = _rigidBodySystem.getContacts();
        const unsigned int numContacts = contacts.size();

        if (numContacts == 0)
        {
            polyscope::removePointCloud("contacts");
        }
        else
        {
            Eigen::MatrixXf contactP(numContacts, 3);
            Eigen::MatrixXf contactN(numContacts, 3);

            for (unsigned int i = 0; i < numContacts; ++i)
            {
                contactP.row(i)(0) = contacts[i]->p(0); contactP.row(i)(1) = contacts[i]->p(1); contactP.row(i)(2) = contacts[i]->p(2);
                contactN.row(i)(0) = contacts[i]->n(0); contactN.row(i)(1) = contacts[i]->n(1); contactN.row(i)(2) = contacts[i]->n(2);
            }

            auto pointCloud = polyscope::registerPointCloud("contacts", contactP);

            pointCloud->setPointColor({ 1.0f, 0.0f, 0.0f });
            pointCloud->setPointRadius(0.005);
            pointCloud->addVectorQuantity("normal", contactN)->setVectorColor({ 1.0f, 1.0f, 0.0f })->setVectorLengthScale(0.05f)->setEnabled(true);
        }
    }

    static void updateJointViz(RigidBodySystem& _rigidBodySystem)
    {
        const auto& joints = _rigidBodySystem.getJoints();
        const unsigned int numJoints = joints.size();

        if (numJoints == 0)
        {
            polyscope::removePointCloud("jointsPoint");
            polyscope::removeCurveNetwork("jointsCurve");
        }
        else
        {
            Eigen::MatrixXf jointP(2 * numJoints, 3);
            Eigen::MatrixXi jointE(numJoints, 2);
            for (unsigned int i = 0; i < numJoints; ++i)
            {
                const Eigen::Vector3f p0 = joints[i]->body0->q * joints[i]->r0 + joints[i]->body0->x;
                const Eigen::Vector3f p1 = joints[i]->body1->q * joints[i]->r1 + joints[i]->body1->x;

                jointP.row(2 * i) = p0;
                jointP.row(2 * i + 1) = p1;
                jointE.row(i) = Eigen::Vector2i(2 * i, 2 * i + 1);
            }

            auto pointCloud = polyscope::registerPointCloud("jointsPoint", jointP);
            pointCloud->setPointColor({ 0.0f, 0.0f, 1.0f });
            pointCloud->setPointRadius(0.005);
            auto curves = polyscope::registerCurveNetwork("jointsCurve", jointP, jointE);
            curves->setRadius(0.002f);
        }
    }


}

SimViewer::SimViewer() :
    m_dt(1.0f / 60.0f), m_subSteps(1), m_dynamicsTime(0.0f),
    m_paused(true), m_stepOnce(false),
    m_enableCollisions(true), m_enableScreenshots(false),
    m_drawContacts(true), m_drawConstraints(true), m_isHydraulic(false),
    m_resetState()
{
    m_resetState = std::make_unique<RigidBodySystemState>(*m_rigidBodySystem);
    reset();
}

SimViewer::~SimViewer()
{
}

void SimViewer::reset()
{
    std::cout << " ---- Reset ----- " << std::endl;
    m_resetState->restore(*m_rigidBodySystem);
    m_dynamicsTime = 0.0f;
    
    hydraulicParams.reset();

    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::save()
{
    std::cout << " ---- Saving current state ----- " << std::endl;
    m_resetState->save(*m_rigidBodySystem);
}

void SimViewer::start()
{
    // Setup Polyscope
    polyscope::options::programName = "slrbs";
    polyscope::options::verbosity = 0;
    polyscope::options::usePrefsFile = false;
    polyscope::options::alwaysRedraw = true;
    polyscope::options::ssaaFactor = 2;
    polyscope::options::openImGuiWindowForUserCallback = true;
    polyscope::options::groundPlaneHeightFactor = 0.0f; // adjust the plane height
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::TileReflection;
    polyscope::options::buildGui = false;
    polyscope::options::maxFPS = -1;
    polyscope::options::groundPlaneEnabled = true;
    polyscope::options::screenshotExtension = ".png";

    // initialize
    polyscope::init();

    // Setup a viewing volume.
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::lengthScale = 10.0f;
    polyscope::state::boundingBox = std::tuple<glm::vec3, glm::vec3>{ {-5., 0, -5.}, {5., 5., 5.} };

    // Specify the update callback
    polyscope::state::userCallback = std::bind(&SimViewer::draw, this);

    // Add pre-step hook.
    m_rigidBodySystem->setPreStepFunc(std::bind(&SimViewer::preStep, this, std::placeholders::_1, std::placeholders::_2));

    // Show the window
    polyscope::show();

}

void SimViewer::drawGUI()
{
    ImGui::Text("Simulation:");
    ImGui::Checkbox("Pause", &m_paused);
    if (ImGui::Button("Step once"))
    {
        m_stepOnce = true;
    }
    if (ImGui::Button("Reset")) {
        reset();
    }
    if (ImGui::Button("Save")) {
        save();
    }

    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("Time step", &m_dt, 0.0f, 0.1f, "%.3f");
    ImGui::SliderInt("Num. sub-steps", &m_subSteps, 1, 20, "%u");
    ImGui::SliderInt("Solver iters.", &(m_rigidBodySystem->solverIter), 1, 100, "%u");
    ImGui::RadioButton("PGS", &(m_rigidBodySystem->solverId), 0);  ImGui::SameLine();
    ImGui::RadioButton("BPP", &(m_rigidBodySystem->solverId), 3);
    ImGui::SliderFloat("Joint stiffness", &Joint::stiffness, 0.0f, 1e9f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Joint damping", &Joint::damping, 0.0f, 1e9f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Contact stiffness", &Contact::stiffness, 0.0f, 1e9f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Contact damping", &Contact::damping, 0.0f, 1e9f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Friction coeff.", &(Contact::mu), 0.0f, 2.0f, "%.2f");

    ImGui::PopItemWidth();


    if (ImGui::Checkbox("Enable collision detecton", &m_enableCollisions)) {
        m_rigidBodySystem->setEnableCollisionDetection(m_enableCollisions);
    }

    ImGui::Checkbox("Draw contacts", &m_drawContacts);
    ImGui::Checkbox("Draw constraints", &m_drawConstraints);
    ImGui::Checkbox("Enable screenshots", &m_enableScreenshots);

    if (ImGui::Button("Sphere on box")) {
        createSphereOnBox();
    }
    if (ImGui::Button("Marble box")) {
        createMarbleBox();
    }
    if (ImGui::Button("Swinging box")) {
        createSwingingBox();
    }
    if (ImGui::Button("Cylinder on plane")) {
        createCylinderOnPlane();
    }
    if (ImGui::Button("Create car scene")) {
        createCarScene();
    }
    if (ImGui::Button("Create piston scene")) {
        createPistonScene();
    }

    ImGui::SliderFloat("x_spool", &(hydraulicParams.x_spool), -0.005f, 0.005f, "%1.4f");
    ImGui::SliderFloat("P_source", &(hydraulicParams.P_source), 1000.0f, 1e9f, "%10.1f");
    ImGui::Text("f_hyd: %8.3f N", hydraulicParams.f_hyd);
    ImGui::Text("P_A: %9.2f Pa", hydraulicParams.P_A);
    ImGui::Text("P_B: %9.2f Pa", hydraulicParams.P_B);
    ImGui::Text("V_A: %9.6f m3", hydraulicParams.V_A);
    ImGui::Text("V_B: %9.6f m3", hydraulicParams.V_B);
    ImGui::Text("Step time: %3.3f ms", m_dynamicsTime);

}

void SimViewer::draw()
{
    drawGUI();

    if( !m_paused || m_stepOnce )
    {


        auto start = std::chrono::high_resolution_clock::now();

        // Step the simulation.
        // The time step dt is divided by the number of sub-steps.
        //
        const float dt = m_dt / (float)m_subSteps;
        for(int i = 0; i < m_subSteps; ++i)
        {
            m_rigidBodySystem->step(dt);
        }
        auto stop = std::chrono::high_resolution_clock::now();

        updateRigidBodyMeshes(*m_rigidBodySystem);

        if (m_drawContacts)
            updateContactPoints(*m_rigidBodySystem);
        else
            polyscope::removePointCloud(strContacts);

        if (m_drawConstraints)
            updateJointViz(*m_rigidBodySystem);
        else
        {
            polyscope::removePointCloud(strJointPoints);
            polyscope::removeCurveNetwork(strJointCurve);
        }


        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_dynamicsTime = (float)duration.count() / 1000.0f;

        if (m_enableScreenshots)
        {
            polyscope::screenshot(false);
        }

        // Clear step-once flag.
        m_stepOnce = false;
    }
}

void SimViewer::createMarbleBox()
{
    Scenarios::createMarbleBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSphereOnBox()
{
    Scenarios::createSphereOnBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSwingingBox()
{
    Scenarios::createSwingingBoxes(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCylinderOnPlane()
{
    Scenarios::createCylinderOnPlane(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCarScene()
{
    Scenarios::createCarScene(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createPistonScene()
{
    Scenarios::createPistonScene(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
    m_isHydraulic = true;
    hydraulicParams.reset();
}


void SimViewer::preStep(float dt, std::vector<RigidBody*>& _bodies)
{
    if (m_isHydraulic)
    {
        auto& bodies = m_rigidBodySystem->getBodies();

        const float L_stroke = 10.0f;
        RigidBody* piston = bodies[0];
        hydraulicParams.v_piston = -1.0f * piston->xdot.x();
        hydraulicParams.x_piston = std::clamp(-1.0f * piston->x.x(), 0.0f, L_stroke);

        const float Cd = 0.62f;
        const float A_x = 0.1f * std::abs(hydraulicParams.x_spool);
        static const float pi = 3.14159f;
        hydraulicParams.V_cyl = pi * hydraulicParams.x_spool + 0.2f;
       
        const float deltaP_A = (hydraulicParams.x_spool > 0) ? (hydraulicParams.P_source - hydraulicParams.P_A) : (hydraulicParams.P_tank - hydraulicParams.P_A);
        const float deltaP_B = (hydraulicParams.x_spool > 0) ? (hydraulicParams.P_tank - hydraulicParams.P_B) : (hydraulicParams.P_source - hydraulicParams.P_B);
        const float Q_A = Cd * A_x * std::sqrt(2.0f * std::abs(deltaP_A) / hydraulicParams.rho) * (deltaP_A > 0.0f ? 1.0f : -1.0f);
        const float Q_B = Cd * A_x * std::sqrt(2.0f * std::abs(deltaP_B) / hydraulicParams.rho) * (deltaP_B > 0.0f ? 1.0f : -1.0f);

        const float A_cap = pi * hydraulicParams.R_piston * hydraulicParams.R_piston;
        const float A_annulus = pi * (hydraulicParams.R_piston * hydraulicParams.R_piston - hydraulicParams.R_rod * hydraulicParams.R_rod);
        const float V_dead = 0.001f;
        hydraulicParams.V_A = V_dead + A_cap * hydraulicParams.x_piston;
        hydraulicParams.V_B = V_dead + A_annulus * (L_stroke - hydraulicParams.x_piston);
        const float dVA_dt = A_cap * hydraulicParams.v_piston;
        const float dVB_dt = -A_annulus * hydraulicParams.v_piston;
        const float dPA_dt = (hydraulicParams.beta / hydraulicParams.V_A) * (Q_A - dVA_dt);
        const float dPB_dt = (hydraulicParams.beta / hydraulicParams.V_B) * (Q_B - dVB_dt);

        hydraulicParams.P_A = std::clamp(hydraulicParams.P_A + dPA_dt * dt, hydraulicParams.P_tank, hydraulicParams.P_relief);
        hydraulicParams.P_B = std::clamp(hydraulicParams.P_B + dPB_dt * dt, hydraulicParams.P_tank, hydraulicParams.P_relief);


        hydraulicParams.f_hyd = (hydraulicParams.P_A * A_cap) - (hydraulicParams.P_B * A_annulus);
        const Eigen::Vector3f f = hydraulicParams.f_hyd * Eigen::Vector3f(-1, 0, 0);
        piston->addForceAtPos({ 0.0f, 0.0f, 0.0f }, f);
    }
}
