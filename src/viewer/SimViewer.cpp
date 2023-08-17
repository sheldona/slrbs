#include "viewer/SimViewer.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/view.h"
#include "imgui.h"

#include <iostream>
#include <functional>

#include "contact/Contact.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/Scenarios.h"

using namespace std;

namespace
{
    static RigidBodySystem* m_rigidBodySystem = new RigidBodySystem;

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

}

SimViewer::SimViewer() :
    m_dt(0.01f), m_subSteps(1),
    m_paused(true), m_stepOnce(false), m_enableCollisions(true)
{

    reset();

}

SimViewer::~SimViewer()
{
}

void SimViewer::reset()
{
    std::cout << " ---- Reset ----- " << std::endl;

    m_rigidBodySystem->clear();
    polyscope::removeAllStructures();
}


void SimViewer::start()
{
    // Setup Polyscope
    polyscope::options::programName = "Rigid Body Tutorial";
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

    // initialize
    polyscope::init();

    // Setup a viewing volume.
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::lengthScale = 10.0f;
    polyscope::state::boundingBox = std::tuple<glm::vec3, glm::vec3>{ {-5., 0, -5.}, {5., 5., 5.} };

    // Specify the update callback
    polyscope::state::userCallback = std::bind(&SimViewer::draw, this);

    // Add pre-step hook.
    m_rigidBodySystem->setPreStepFunc(std::bind(&SimViewer::preStep, this, std::placeholders::_1));

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

    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("Time step", &m_dt, 0.0f, 0.1f, "%.3f");
    ImGui::SliderInt("Num. sub-steps", &m_subSteps, 1, 20, "%u");
    ImGui::SliderInt("Solver iters.", &(m_rigidBodySystem->solverIter), 1, 100, "%u");
    ImGui::SliderFloat("Friction coeff.", &(Contact::mu), 0.0f, 2.0f, "%.2f");
    ImGui::PopItemWidth();

    if (ImGui::Checkbox("Enable collision detecton", &m_enableCollisions)) {
        m_rigidBodySystem->setEnableCollisionDetection(m_enableCollisions);
    }

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

}

void SimViewer::draw()
{
    drawGUI();

    if( !m_paused || m_stepOnce )
    {
        // Step the simulation.
        // The time step dt is divided by the number of sub-steps.
        //
        const float dt = m_dt / (float)m_subSteps;
        for(int i = 0; i < m_subSteps; ++i)
        {
            m_rigidBodySystem->step(dt);
        }

        updateRigidBodyMeshes(*m_rigidBodySystem);
        updateContactPoints(*m_rigidBodySystem);

        // Clear step-once flag.
        m_stepOnce = false;
    }
}

void SimViewer::createMarbleBox()
{
    Scenarios::createMarbleBox(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
}

void SimViewer::createSphereOnBox()
{
    Scenarios::createSphereOnBox(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
}

void SimViewer::createSwingingBox()
{
    Scenarios::createSwingingBox(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
}

void SimViewer::createCylinderOnPlane()
{
    Scenarios::createCylinderOnPlane(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
}

void SimViewer::createCarScene()
{
    Scenarios::createCarScene(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
}

void SimViewer::preStep(std::vector<RigidBody*>& _bodies)
{
    // do something useful here?
}
