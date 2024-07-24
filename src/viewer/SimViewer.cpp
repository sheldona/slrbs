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
#include "log/CSVLogger.h"

using namespace std;

namespace
{
    static RigidBodySystem* m_rigidBodySystem = new RigidBodySystem;
    static CSVLogger s_log;
    static int s_substepsIdx = -1;
    static int s_kinEnergyIdx = -1;
    
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


}

SimViewer::SimViewer() :
    m_adaptiveTimesteps(false),
    m_gsDamping(false),
    m_alpha(0.01f),
    m_dt(0.01f), m_subSteps(1), 
    m_dynamicsTime(0.0f), m_frameCounter(0),
    m_paused(true), m_stepOnce(false),
    m_enableCollisions(true), m_enableScreenshots(false),
    m_enableLogging(false),
    m_drawContacts(true), m_drawConstraints(true),
    m_resetState()
{
    s_kinEnergyIdx = s_log.addField("kin_energy");
    s_substepsIdx = s_log.addField("substeps");

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
    m_frameCounter = 0;

    s_log.clear();

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

    polyscope::view::windowWidth = 1980;
    polyscope::view::windowHeight = 1080;

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
    ImGui::Checkbox("Adaptive time steps", &m_adaptiveTimesteps);
    ImGui::Checkbox("Geometric stiffness damping", &m_gsDamping);
    if (m_adaptiveTimesteps || m_gsDamping)
        ImGui::SliderFloat("Alpha", &m_alpha, 0.0f, 1.0f);
    ImGui::SliderFloat("Time step", &m_dt, 0.0f, 0.1f, "%.3f");
    ImGui::SliderInt("Num. sub-steps", &m_subSteps, 1, 20, "%u");
    ImGui::SliderInt("Solver iters.", &(m_rigidBodySystem->solverIter), 1, 100, "%u");
    ImGui::SliderFloat("Friction coeff.", &(Contact::mu), 0.0f, 2.0f, "%.2f");
    ImGui::RadioButton("PGS", &(m_rigidBodySystem->solverId), 0);  ImGui::SameLine();
    ImGui::RadioButton("Conj. Gradient (NO CONTACT)", &(m_rigidBodySystem->solverId), 1);
    ImGui::RadioButton("Conj. Residual (NO CONTACT)", &(m_rigidBodySystem->solverId), 2);
    ImGui::RadioButton("BPP", &(m_rigidBodySystem->solverId), 3);
    ImGui::PopItemWidth();

    if (ImGui::Checkbox("Enable collision detecton", &m_enableCollisions)) {
        m_rigidBodySystem->setEnableCollisionDetection(m_enableCollisions);
    }

    ImGui::Checkbox("Draw contacts", &m_drawContacts);
    ImGui::Checkbox("Draw constraints", &m_drawConstraints);
    ImGui::Checkbox("Enable screenshots", &m_enableScreenshots);

    if (ImGui::Button("Save log")) {
        s_log.save("log.csv");
    }
    ImGui::Checkbox("Enable logging", &m_enableLogging);

    if (ImGui::Button("Sphere on box")) {
        createSphereOnBox();
    }
    if (ImGui::Button("Marble box")) {
        createMarbleBox();
    }
    if (ImGui::Button("Swinging box")) {
        createSwingingBox();
    }
    if (ImGui::Button("Box stack")) {
        createBoxStack();
    }
    if (ImGui::Button("Cylinder on plane")) {
        createCylinderOnPlane();
    }
    if (ImGui::Button("Create car scene")) {
        createCarScene();
    }
    if (ImGui::Button("Create bridge scene")) {
        createBridgeScene();
    }

    ImGui::Text("Step time: %3.3f ms", m_dynamicsTime);
    ImGui::Text("Frame: %d ", m_frameCounter);

}

void SimViewer::draw()
{
    drawGUI();

    if( !m_paused || m_stepOnce )
    {
        auto start = std::chrono::high_resolution_clock::now();

        // Automatically choose next substeps based on geometric stiffness
        if (m_adaptiveTimesteps)
        {
            for (auto b : m_rigidBodySystem->getBodies())
                b->gsSum.setZero();

            // TODO: need to use 12x12 blocks for g.s. matrix, otherwise off-diagonal terms
            // are overlooked when computing the col norm.
            //

            for (auto j : m_rigidBodySystem->getJoints())
            {
                j->computeGeometricStiffness();
                j->body0->gsSum += j->G0;
                j->body1->gsSum += j->G1;
            }

            float maxK_M = 0.0f;
            for (auto b : m_rigidBodySystem->getBodies())
            {
                if (b->fixed) continue;

                for (int c = 0; c < 3; c++)
                {
                    const float m = b->mass;
                    const float k = b->gsSum.col(c).norm();
                    maxK_M = std::max(k / m, maxK_M);
                }

                for (int c = 0; c < 3; c++)
                {
                    const float m = b->I(c, c);
                    const float k = b->gsSum.col(c + 3).norm();
                    maxK_M = std::max(k / m, maxK_M);
                }
            }

            const float dt = 2.0f * m_alpha * std::sqrt(1.0f / maxK_M);
            m_subSteps = std::max(1, (int)ceil(m_dt / dt));
        }

        const float dt = m_dt / (float)m_subSteps;


        // Step the simulation.
        // The time step dt is divided by the number of sub-steps.
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

        const float kinEnergy = computeKineticEnergy(*m_rigidBodySystem);
        s_log.pushVal(s_kinEnergyIdx, kinEnergy);
        s_log.pushVal(s_substepsIdx, (float)m_subSteps);
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

void SimViewer::createBoxStack()
{
    Scenarios::createBoxStack(*m_rigidBodySystem);
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

void SimViewer::createBridgeScene()
{
    Scenarios::createRopeBridgeScene(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::preStep(RigidBodySystem& rigidBodySystem, float h)
{
    auto& bodies = rigidBodySystem.getBodies();
    auto& joints = rigidBodySystem.getJoints();

    // Reset geometric stiffness damping values.
    //
    for (auto b : bodies)
        b->gsDamp.setZero();

    // Recompute geometric stiffness damping.
    // The damping will be added to the bodies' inertia matrices during integration.
    //
    if (m_gsDamping)
    {
        for (auto j : joints)
        {
            j->computeGeometricStiffness();
            j->body0->gsSum += j->G0;
            j->body1->gsSum += j->G1;
        }

        for (auto b : m_rigidBodySystem->getBodies())
        {
            if (b->fixed)
                continue;

            for (int c = 0; c < 3; c++)
            {
                const float m = b->I(c, c);
                const float k = 2.0f * b->gsSum.col(c + 3).norm();
                b->gsDamp(c) = std::max(0.0f, h * h * k - 4 * m_alpha * m);
            }
            // SA: seems we have to do this again here so that 
            //  solver has most recent gs damping information
            b->updateInertiaMatrix();
        }
    }
}
