#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include <polyscope/polyscope.h>
#include "rigidbody/Scenarios.h"

#include "joint/Spherical.h"
#include "joint/Hinge.h"
#include "joint/Distance.h"
#include "joint/Prismatic.h"
#include "util/Types.h"
#include <collision/Geometry.h>
#include <Eigen/Dense>
#include <map>
#include <iostream>
#include <cstdlib>

#include <polyscope/surface_mesh.h>

#ifndef M_PI
    const float M_PI = 3.14159265358979323846f;
#endif

using namespace std;

vector<Scenarios::ScenarioInfo> Scenarios::getAvailableScenarios() {
    return {
        {MARBLE_BOX, "Marble Box", "Box filled with marble spheres", "", createMarbleBox},
        {SPHERE_ON_BOX, "Sphere on Box", "Simple sphere falling onto a box", "", createSphereOnBox},
        {SWINGING_BOXES, "Swinging Boxes", "Chain of boxes connected by hinges", "", createSwingingBoxes},
        {CYLINDER_ON_PLANE, "Cylinder on Plane", "Cylinder-plane collision test", "", createCylinderOnPlane},
        {CAR_SCENE, "Car Scene", "Car with chassis and wheels", "", createCarScene},
        {STACK, "Stack", "Stack of objects under pressure", "", createStack},
        {ROPE_BRIDGE, "Rope Bridge", "Flexible bridge with a rolling sphere", "", createRopeBridgeScene},
        {SPHERE_SPHERE_DISTANCE, "Sphere-Sphere Distance", "Distance constraint between spheres", "", createSphereSphereDistance},
        {SPHERE_INSIDE_BOX, "Sphere in Box", "Sphere constrained inside a box", "", createSphereInsideBox},
        {BOX_ON_PLANE, "Box on Plane", "Box resting on an angled plane", "", createBoxOnPlane},
        {CYLINDER_SPHERE_TEST, "Cylinder-Sphere Test", "Sphere falling onto a cylinder", "",
            [](RigidBodySystem& sys){ createCylinderSphereTest(sys, Eigen::AngleAxisf(0.0f, Eigen::Vector3f(0,0,1))); }
        },
        {ROPE_LADDER, "Rope Ladder", "Flexible ladder with distance constraints", "", createRopeLadder},
        {CUSTOM_DOUBLE_PENDULUM, "Double Pendulum", "Custom pendulum setup with distance joints", "", createCustomScenario},
        {CUSTOM_SPHERICAL_JOINT, "Spherical Joint Test", "Spheres connected by a spherical joint", "", createCustomScenario2},
        {CUSTOM_TWO_PRISMATICS, "Two Prismatics", "Sphere constrained between two prismatic joints", "", createCustomScenario3},
        {CUSTOM_BOX_SPHERE_JOINT, "Box-Sphere Joint", "Box connected to a sphere with a spherical joint", "", createCustomScenario4},
        {CUSTOM_THREE_PRISMATICS, "Three-Axis Constraint", "Sphere constrained along three axes", "",
            [](RigidBodySystem& sys){ createCustomScenario5(sys, 0.0f, 7.0f, 0.0f); }
        },
        {CUSTOM_HINGE_JOINT, "Hinge Joint Test", "Box connected with a hinge joint", "", createCustomScenario6},
        {CUSTOM_TENSILE_TABLE, "Tensile Table", "Tensile structure with platforms and cables", "", createCustomScenario7},
        {CUSTOM_HOLLOW_BOX, "Hollow Box & Spheres", "Multiple spheres inside a hollow box framework", "", createCustomScenario8},
        {CUSTOM_BOX_WITH_FACES, "Box with Faces", "Box with rendered faces containing multiple spheres", "", createCustomScenario9}
    };
}

bool Scenarios::createScenario(RigidBodySystem& system, ScenarioID id) {
    for (auto& info : getAvailableScenarios()) {
        if (info.id == id) {
            info.createFunction(system);
            return true;
        }
    }
    return false;
}

Scenarios::ScenarioInfo Scenarios::getScenarioInfo(ScenarioID id) {
    for (auto& info : getAvailableScenarios()) {
        if (info.id == id) return info;
    }
    // fallback
    return {MARBLE_BOX, "Unknown", "Unknown scenario", "", createMarbleBox};
}


// Box filled with balls.
//
void Scenarios::createMarbleBox(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading marble box scenario" << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    const float radius = 0.5f;
    // Create two layers of "marbles", in a grid layout.
    //
    for (int i = 0; i < 9; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            RigidBody* body1 = new RigidBody(1.0f, new Sphere(radius), createSphere(radius));
            body1->id = bodyId;
            body1->x = {-4.0f + (float)i * 1.0f, 2.0f, -4.0f + (float)j * 1.0f};
            body1->xdot = Eigen::Vector3f::Random();
            bodyMap[bodyId++] = body1;
            rigidBodySystem.addBody(body1);
            body1->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f});
            body1->mesh->setTransparency(0.6f);

            RigidBody* body2 = new RigidBody(1.0f, new Sphere(radius), createSphere(radius));
            body2->id = bodyId;
            body2->x = {-4.0f + (float)i * 1.0f, 3.0f, -4.0f + (float)j * 1.0f};
            body2->xdot = Eigen::Vector3f::Random();
            bodyMap[bodyId++] = body2;
            rigidBodySystem.addBody(body2);
            body2->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f});
            body2->mesh->setTransparency(0.6f);
        }
    }

    // Create the box to hold the marbles.
    const Eigen::Vector3f sideDim(Eigen::Vector3f(0.4f, 4.0f, 10.0f));
    const Eigen::Vector3f botDim(Eigen::Vector3f(10.0f, 0.4f, 10.0f));

    int boxId0 = bodyId++;
    int boxId1 = bodyId++;
    int boxId2 = bodyId++;
    int boxId3 = bodyId++;
    int boxId4 = bodyId++;

    RigidBody* body0 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim));
    body0->id = boxId0;
    RigidBody* body1 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim));
    body1->id = boxId1;
    RigidBody* body2 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim));
    body2->id = boxId2;
    RigidBody* body3 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim));
    body3->id = boxId3;
    RigidBody* body4 = new RigidBody(1.0f, new Box(botDim), createBox(botDim));
    body4->id = boxId4;

    bodyMap[boxId0] = body0;
    bodyMap[boxId1] = body1;
    bodyMap[boxId2] = body2;
    bodyMap[boxId3] = body3;
    bodyMap[boxId4] = body4;

    body0->fixed = true;
    body1->fixed = true;
    body2->fixed = true;
    body3->fixed = true;
    body4->fixed = true;
    body0->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);
    body1->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);
    body2->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);
    body3->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);
    body4->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);
    body0->x = {4.75f, 2.0f, 0.0f};
    body1->x = {-4.75f, 2.0f, 0.0f};
    body2->x = {0.0f, 2.0f, 4.75f};
    body2->q = Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 1, 0));
    body3->x = {0.0f, 2.0f, -4.75f};
    body3->q = Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 1, 0));
    body4->x = {0.0f, 0.0f, 0.0f};

    rigidBodySystem.addBody(body0);
    rigidBodySystem.addBody(body1);
    rigidBodySystem.addBody(body2);
    rigidBodySystem.addBody(body3);
    rigidBodySystem.addBody(body4);
}

// Simple sphere falling on a box.
//
void Scenarios::createSphereOnBox(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading sphere-on-box scenario." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;

    // Create a sphere.
    const float radius = 0.5f;
    RigidBody* bodySphere = new RigidBody(1.0f, new Sphere(radius), createSphere(radius));
    bodySphere->x.y() = 4.0f;
    bodySphere->omega = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
    bodySphere->mesh->setTransparency(0.8f);

    // Create a box that will act as the ground.
    const Eigen::Vector3f dim(10.0f, 0.4f, 10.0f);
    RigidBody* bodyBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
    bodyBox->fixed = true;

    // Assign IDs
    int sphereId = 0;
    int boxId = 1;

    bodySphere->id = sphereId;
    bodyBox->id = boxId;

    bodyMap[sphereId] = bodySphere;
    bodyMap[boxId] = bodyBox;

    rigidBodySystem.addBody(bodySphere);
    rigidBodySystem.addBody(bodyBox);

    bodySphere->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(1.0f);
    bodyBox->mesh->setSurfaceColor({0.2f, 0.2f, 0.2f})->setSmoothShade(false)->setTransparency(0.4f);
}

// Box hanging from a box
//
void Scenarios::createSwingingBoxes(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading swinging boxes scenario." << std::endl;

    const int N = 20;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;

    // Create a box.
    const Eigen::Vector3f dim({1.0f, 1.0f, 1.0f});
    RigidBody* topBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
    topBox->x = {0.0f, 1.5f*(float)N, 0.0f};
    topBox->fixed = true;

    int topBoxId = 0;
    topBox->id = topBoxId;
    bodyMap[topBoxId] = topBox;
    rigidBodySystem.addBody(topBox);

    const Eigen::Vector3f dx(0.0f, 1.5f, 0.0f);
    RigidBody* parent = topBox;
    int parentId = topBoxId;

    for (int i = 0; i < N-1; ++i)
    {
        // Create the next box in the chain.
        RigidBody* nextBox = nullptr;
        if (i == (N - 2)) nextBox = new RigidBody(100.0f, new Box(dim), createBox(dim));
        else nextBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
        nextBox->x = parent->x - dx;

        int nextBoxId = i + 1;
        nextBox->id = nextBoxId;
        bodyMap[nextBoxId] = nextBox;

        // Add a hinge between parent->nextBox
        Joint* j = new Hinge(bodyMap[parentId], bodyMap[nextBoxId], -0.5f * dx, Eigen::Quaternionf::Identity(), 0.5f * dx, Eigen::Quaternionf::Identity());

        // Add new box and hinge to the rigid body system.
        rigidBodySystem.addBody(nextBox);
        rigidBodySystem.addJoint(j);
        parent = nextBox;
        parentId = nextBoxId;
    }

    parent->xdot = {0.0f, 0.0, 10.0f};
}

// Cylinder-plane collision test
//
void Scenarios::createCylinderOnPlane(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading cylinder on plane scenario." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;

    // Create a cylinder.
    const float height = 2.0f;
    const float radius = 1.0f;
    RigidBody* cyl = new RigidBody(1.0f, new Cylinder(height, radius), createCylinder(16, radius, height));
    cyl->x = {0.0f, 2.0f, 0.0f};
    cyl->q = Eigen::AngleAxisf(0.57f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

    // Create a ground plane.
    RigidBody* plane = new RigidBody(1.0f, new Plane({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), "");
    plane->x = {0.0f, 0.0f, 0.0f};
    plane->fixed = true;

    // Assign IDs
    int cylId = 0;
    int planeId = 1;

    cyl->id = cylId;
    plane->id = planeId;

    bodyMap[cylId] = cyl;
    bodyMap[planeId] = plane;

    rigidBodySystem.addBody(cyl);
    rigidBodySystem.addBody(plane);
}

// Car scene with wheels and chassis
//
void Scenarios::createCarScene(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading car scenario." << std::endl;

    // Create a car.
    RigidBody* chassis = new RigidBody(5.0f, new Box({2.0f, 0.5f, 3.0f}), createBox({2.0f, 0.5f, 3.0f}));
    RigidBody* lfwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));
    RigidBody* rfwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));
    RigidBody* lrwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));
    RigidBody* rrwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));

    // Set position and orientation
    chassis->x = {0.0f, 0.5f, 0.0f};
    chassis->xdot = {0.0f, 0.0f, -10.0f};
    lfwheel->x = {1.0f, 0.5f, 1.5f};
    rfwheel->x = {-1.0f, 0.5f, 1.5f};
    lrwheel->x = {1.0f, 0.5f, -1.5f};
    rrwheel->x = {-1.0f, 0.5f, -1.5f};
    lfwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    rfwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    lrwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    rrwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

    // Create a ground plane.
    RigidBody* plane = new RigidBody(1.0f, new Plane({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), "");
    plane->x = {0.0f, 0.0f, 0.0f};
    plane->fixed = true;

    // Add bodies to the system with explicit IDs
    int chassisId = 0;
    int lfwheelId = 1;
    int rfwheelId = 2;
    int lrwheelId = 3;
    int rrwheelId = 4;
    int planeId = 5;

    chassis->id = chassisId;
    lfwheel->id = lfwheelId;
    rfwheel->id = rfwheelId;
    lrwheel->id = lrwheelId;
    rrwheel->id = rrwheelId;
    plane->id = planeId;

    // Store the ID-to-body mapping for joint creation
    std::map<int, RigidBody*> bodyMap;
    bodyMap[chassisId] = chassis;
    bodyMap[lfwheelId] = lfwheel;
    bodyMap[rfwheelId] = rfwheel;
    bodyMap[lrwheelId] = lrwheel;
    bodyMap[rrwheelId] = rrwheel;
    bodyMap[planeId] = plane;

    // Add bodies to the system with sequential indices
    rigidBodySystem.addBody(chassis);   // 0
    rigidBodySystem.addBody(lfwheel);   // 1
    rigidBodySystem.addBody(rfwheel);   // 2
    rigidBodySystem.addBody(lrwheel);   // 3
    rigidBodySystem.addBody(rrwheel);   // 4
    rigidBodySystem.addBody(plane);     // 5

    // Setup the vehicle joints/constraints.
    Hinge* lfhinge = new Hinge(bodyMap[chassisId], bodyMap[lfwheelId],
        {1.0f, 0.0f, 1.5f},
        Eigen::Quaternionf::Identity(),
        {0.0f, 0.0f, 0.0f},
        Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
    );

    Hinge* rfhinge = new Hinge(bodyMap[chassisId], bodyMap[rfwheelId],
        {-1.0f, 0.0f, 1.5f},
        Eigen::Quaternionf::Identity(),
        {0.0f, 0.0f, 0.0f},
        Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
    );

    Hinge* lrhinge = new Hinge(bodyMap[chassisId], bodyMap[lrwheelId],
        {1.0f, 0.0f, -1.5f},
        Eigen::Quaternionf::Identity(),
        {0.0f, 0.0f, 0.0f},
        Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
    );

    Hinge* rrhinge = new Hinge(bodyMap[chassisId], bodyMap[rrwheelId],
        {-1.0f, 0.0f, -1.5f},
        Eigen::Quaternionf::Identity(),
        {0.0f, 0.0f, 0.0f},
        Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
    );

    rigidBodySystem.addJoint(lfhinge);
    rigidBodySystem.addJoint(rfhinge);
    rigidBodySystem.addJoint(lrhinge);
    rigidBodySystem.addJoint(rrhinge);
}

void Scenarios::createStack(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading stack scenario." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    // Create a box that will act as the ground.
    RigidBody* bodyBox = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 1.0f, 10.0f)), createBox(Eigen::Vector3f(10, 1.0f, 10)));
    bodyBox->id = bodyId;
    bodyBox->fixed = true;
    bodyBox->mesh->setSurfaceColor({0.2f, 0.2f, 0.2f})->setSmoothShade(false)->setTransparency(0.4f);
    bodyBox->mesh->setEdgeWidth(0.0f);

    bodyMap[bodyId++] = bodyBox;
    rigidBodySystem.addBody(bodyBox);

    const int N = 4;
    for (int i = 1; i <= N; i++)
    {
        RigidBody* body1 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
        body1->id = bodyId;
        body1->x = {-4.0f, 1.5f * i - 0.5f, -4.0f};
        bodyMap[bodyId++] = body1;
        rigidBodySystem.addBody(body1);
        body1->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(0.0f);
        body1->mesh->setTransparency(0.8f);
        body1->mesh->setSmoothShade(true);

        RigidBody* body2 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
        body2->id = bodyId;
        body2->x = {4.0f, 1.5f * i - 0.5f, -4.0f};
        bodyMap[bodyId++] = body2;
        rigidBodySystem.addBody(body2);
        body2->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(0.0f);
        body2->mesh->setTransparency(0.8f);
        body2->mesh->setSmoothShade(true);

        RigidBody* body3 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
        body3->id = bodyId;
        body3->x = {-4.0f, 1.5f * i - 0.5f, 4.0f};
        bodyMap[bodyId++] = body3;
        rigidBodySystem.addBody(body3);
        body3->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(0.0f);
        body3->mesh->setTransparency(0.8f);
        body3->mesh->setSmoothShade(true);

        RigidBody* body4 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
        body4->id = bodyId;
        body4->x = {4.0f, 1.5f * i - 0.5f, 4.0f};
        bodyMap[bodyId++] = body4;
        rigidBodySystem.addBody(body4);
        body4->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(0.0f);
        body4->mesh->setTransparency(0.8f);
        body4->mesh->setSmoothShade(true);

        if (i < N)
        {
            RigidBody* body5 = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.5f, 10.0f)), createBox(Eigen::Vector3f(10, 0.5f, 10)));
            body5->id = bodyId;
            body5->x = {0, 1.5f * i + 0.25f, 0};
            bodyMap[bodyId++] = body5;
            rigidBodySystem.addBody(body5);
            body5->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f})->setEdgeWidth(0.0f);
            body5->mesh->setTransparency(0.8f);
        }
    }

    RigidBody* topBox = new RigidBody(20000.0f, new Box(Eigen::Vector3f(15.0f, 1.0f, 15.0f)), createBox(Eigen::Vector3f(15.0f, 1.0f, 15.0f)));
    topBox->id = bodyId;
    topBox->x = {0, 1.5f * N + 0.5f, 0};
    bodyMap[bodyId++] = topBox;
    rigidBodySystem.addBody(topBox);
    topBox->mesh->setSurfaceColor({0.1f, 0.2f, 1.0f})->setEdgeWidth(0.0f);
    topBox->mesh->setTransparency(0.8f);
}

void Scenarios::createRopeBridgeScene(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading bridge scenario." << std::endl;

    const int N = 20;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    const float dx = 0.6f;
    const float y = 3.0f;
    const float x0 = -(N / 2) * dx;
    float x = x0;

    // Create a box.
    const Eigen::Vector3f dim({0.5f, 0.1f, 1.0f});
    RigidBody* firstBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
    firstBox->id = bodyId;
    firstBox->x = {x, y, 0.0f};
    firstBox->fixed = true;

    int firstBoxId = bodyId++;
    bodyMap[firstBoxId] = firstBox;
    rigidBodySystem.addBody(firstBox);
    firstBox->mesh->setSurfaceColor({1.0f, 1.0f, 0.1f})->setEdgeWidth(0.0f);

    RigidBody* parent = firstBox;
    int parentId = firstBoxId;

    for (int i = 0; i < N - 1; ++i)
    {
        // Create the next box in the chain.
        x += dx;
        RigidBody* nextBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
        nextBox->id = bodyId;
        nextBox->x = {x, y, 0.0f};
        nextBox->mesh->setSurfaceColor({0.1f, 0.2f, 1.0f})->setEdgeWidth(0.0f);

        // Store box in map
        int nextBoxId = bodyId++;
        bodyMap[nextBoxId] = nextBox;

        // Add new box
        rigidBodySystem.addBody(nextBox);

        // Add spherical joints between parent->nextBox
        Joint* j0 = new Spherical(bodyMap[parentId], bodyMap[nextBoxId], {dx / 2.0f, 0.0f, 0.5f}, {-dx / 2.0f, 0.0f, 0.5f});
        Joint* j1 = new Spherical(bodyMap[parentId], bodyMap[nextBoxId], {dx / 2.0f, 0.0f, -0.5f}, {-dx / 2.0f, 0.0f, -0.5f});

        // Add spherical joints to the rigid body system.
        rigidBodySystem.addJoint(j0);
        rigidBodySystem.addJoint(j1);
        parent = nextBox;
        parentId = nextBoxId;
    }
    parent->fixed = true;
    parent->mesh->setSurfaceColor({1.0f, 1.0f, 0.1f});

    // Create a sphere.
    const float radius = 0.5f;
    RigidBody* bodySphere = new RigidBody(1000.0f, new Sphere(radius), createSphere(radius));
    bodySphere->id = bodyId;
    bodySphere->x = {x0, y + 1.0f, 0.0f};
    bodySphere->omega = {0.0f, 0.0f, -5.0f};
    bodySphere->xdot = {1.0f, 0.0f, 0.0f};
    bodySphere->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(0.0f)->setTransparency(0.8f);
    bodySphere->mesh->setSmoothShade(true);

    int sphereId = bodyId++;
    bodyMap[sphereId] = bodySphere;
    rigidBodySystem.addBody(bodySphere);
}

void Scenarios::createSphereSphereDistance(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading sphere-sphere distance test." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    RigidBody* sphere1 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sphere1->id = bodyId;
    sphere1->x = {0.0f, 4.0f, 0.0f};
    sphere1->fixed = true;

    RigidBody* sphere2 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sphere2->id = bodyId+1;
    sphere2->x = {-2.0f, 4.0f, 0.0f};

    int sphere1Id = bodyId++;
    int sphere2Id = bodyId++;

    bodyMap[sphere1Id] = sphere1;
    bodyMap[sphere2Id] = sphere2;

    rigidBodySystem.addBody(sphere1);
    rigidBodySystem.addBody(sphere2);

    Distance* joint = new Distance(bodyMap[sphere1Id], bodyMap[sphere2Id], {0.0f, -0.5f, 0.0f}, {0.0f, -0.5f, 0.0f}, 2.0f);
    rigidBodySystem.addJoint(joint);
}

void Scenarios::createSphereInsideBox(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading sphere-inside-box test." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    const Eigen::Vector3f dim(2.0f, 2.0f, 2.0f);
    RigidBody* box = new RigidBody(1.0f, new Box(dim), createBox(dim));
    box->id = bodyId;
    box->x = {0.0f, 2.0f, 0.0f};
    box->fixed = true;
    box->mesh->setTransparency(0.4f);

    RigidBody* sphere = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sphere->id = bodyId+1;
    sphere->x = {0.0f, 2.5f, 0.0f};
    sphere->mesh->setTransparency(0.6f);

    int boxId = bodyId++;
    int sphereId = bodyId++;

    bodyMap[boxId] = box;
    bodyMap[sphereId] = sphere;

    rigidBodySystem.addBody(box);
    rigidBodySystem.addBody(sphere);
}

void Scenarios::createBoxOnPlane(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading box-on-plane scenario." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    const Eigen::Vector3f dim(1.0f, 1.0f, 1.0f);
    RigidBody* bodyBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
    bodyBox->id = bodyId;
    bodyBox->x = {-1.0f, 2.0f, 0.0f};
    bodyBox->q = Eigen::AngleAxisf(-0.5236, Eigen::Vector3f(0, 0, 1));

    const Eigen::Vector3f n({0.5f, 0.866f, 0.0f});
    const Eigen::Vector3f p({0.0f, 0.0f, 0.0f});
    RigidBody* plane = new RigidBody(1.0f, new Plane(p, n), createPlane(p, n));
    plane->id = bodyId+1;
    plane->fixed = true;

    int boxId = bodyId++;
    int planeId = bodyId++;

    bodyMap[boxId] = bodyBox;
    bodyMap[planeId] = plane;

    rigidBodySystem.addBody(bodyBox);
    rigidBodySystem.addBody(plane);

    bodyBox->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(1.0f)->setTransparency(0.6f);
    plane->mesh->setSurfaceColor({0.2f, 0.2f, 0.2f})->setSmoothShade(false)->setTransparency(0.4f);
}

void Scenarios::createCylinderSphereTest(RigidBodySystem &rigidBodySystem, const Eigen::AngleAxisf &aa)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading cylinder on sphere scenario." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    RigidBody* bodySphere = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    bodySphere->id = bodyId;
    bodySphere->x = {0.0f, 4.0f, 0.0f};
    bodySphere->omega = {0.0f, 0.0f, 1.0f};
    bodySphere->mesh->setTransparency(0.8f);

    RigidBody* bodyCyl = new RigidBody(1.0f, new Cylinder(2.0f, 0.5f), createCylinder(16, 0.5f, 2.0f));
    bodyCyl->id = bodyId+1;
    bodyCyl->x = {0.0f, 1.0f, 0.0f};
    bodyCyl->q = Eigen::Quaternionf(aa);
    bodyCyl->fixed = true;

    int sphereId = bodyId++;
    int cylId = bodyId++;

    bodyMap[sphereId] = bodySphere;
    bodyMap[cylId] = bodyCyl;

    rigidBodySystem.addBody(bodySphere);
    rigidBodySystem.addBody(bodyCyl);

    bodySphere->mesh->setSurfaceColor({0.1f, 1.0f, 0.2f})->setEdgeWidth(1.0f)->setTransparency(0.4f);
    bodyCyl->mesh->setSurfaceColor({0.1f, 0.1f, 1.0f})->setSmoothShade(false)->setTransparency(0.4f);
}

void Scenarios::createRopeLadder(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading rope ladder scenario." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    const int N = 8;
    const Eigen::Vector3f dim = {1.0f, 0.5f, 0.6f};
    const float dy = 0.8f;

    RigidBody* topBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
    topBox->id = bodyId;
    topBox->x = {0.0f, dy * (float)N, 0.0f};
    topBox->fixed = true;

    int topBoxId = bodyId++;
    bodyMap[topBoxId] = topBox;
    rigidBodySystem.addBody(topBox);

    RigidBody* parent = topBox;
    int parentId = topBoxId;

    for (int i = 0; i < N - 1; ++i)
    {
        RigidBody* nextBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
        nextBox->id = bodyId;
        nextBox->x = parent->x - Eigen::Vector3f(0.0f, dy, 0.0f);

        int nextBoxId = bodyId++;
        bodyMap[nextBoxId] = nextBox;
        rigidBodySystem.addBody(nextBox);

        rigidBodySystem.addJoint(new Distance(bodyMap[parentId], bodyMap[nextBoxId], {0.5f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, dy));
        rigidBodySystem.addJoint(new Distance(bodyMap[parentId], bodyMap[nextBoxId], {-0.5f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f}, dy));

        parent = nextBox;
        parentId = nextBoxId;
    }

    parent->xdot = {0.0f, 0.0, 5.0f};
}

void Scenarios::createCustomScenario(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading custom scenario: double pendulum." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    RigidBody* body1 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    body1->id = bodyId;
    RigidBody* body2 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    body2->id = bodyId+1;
    RigidBody* body3 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.5f, 0.5f, 0.5f)), createBox(Eigen::Vector3f(0.5f, 0.5f, 0.5f)));
    body3->id = bodyId+2;

    body1->x = {0.0f, 6.0f, 0.0f};
    body2->x = {2.0f, 6.0f, 0.0f};
    body3->x = {4.0f, 6.0f, 0.0f};

    int body1Id = bodyId++;
    int body2Id = bodyId++;
    int body3Id = bodyId++;

    bodyMap[body1Id] = body1;
    bodyMap[body2Id] = body2;
    bodyMap[body3Id] = body3;

    rigidBodySystem.addBody(body1);
    rigidBodySystem.addBody(body2);
    rigidBodySystem.addBody(body3);

    rigidBodySystem.addJoint(new Distance(bodyMap[body1Id], bodyMap[body2Id], {0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 1.0f));
    rigidBodySystem.addJoint(new Distance(bodyMap[body2Id], bodyMap[body3Id], {0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}, 1.0f));

    body1->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f});
    body2->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f});
    body3->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f});

    body1->mesh->setTransparency(0.6f);
    body2->mesh->setTransparency(0.6f);
    body3->mesh->setTransparency(0.6f);

    body1->fixed = true;

    RigidBody* body0 = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, -1.0f, 10.0f)), createBox(Eigen::Vector3f(10.0f, 0.4f, 10.0f)));
    body0->id = bodyId;
    body0->fixed = true;

    int body0Id = bodyId++;
    bodyMap[body0Id] = body0;
    rigidBodySystem.addBody(body0);

    body0->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);
}

void Scenarios::createCustomScenario2(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading custom scenario: sphere spherical joint." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    RigidBody* sphere1 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sphere1->id = bodyId;
    RigidBody* sphere2 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sphere2->id = bodyId+1;

    sphere1->x = {1.0f, 6.0f, 0.0f};
    sphere2->xdot = {0.0f, 0.0f, 5.0f};
    sphere2->x = {2.0f, 6.0f, 0.0f};
    sphere2->omega = {0.0f, 0.0f, -5.0f};

    int sphere1Id = bodyId++;
    int sphere2Id = bodyId++;

    bodyMap[sphere1Id] = sphere1;
    bodyMap[sphere2Id] = sphere2;

    rigidBodySystem.addBody(sphere1);
    rigidBodySystem.addBody(sphere2);

    Spherical* joint = new Spherical(bodyMap[sphere1Id], bodyMap[sphere2Id], {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    rigidBodySystem.addJoint(joint);

    sphere1->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f});
    sphere2->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f});

    sphere1->mesh->setTransparency(0.6f);
    sphere2->mesh->setTransparency(0.6f);

    RigidBody* body0 = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, -1.0f, 10.0f)), createBox(Eigen::Vector3f(10.0f, 0.4f, 10.0f)));
    body0->id = bodyId;
    body0->fixed = true;

    int body0Id = bodyId++;
    bodyMap[body0Id] = body0;
    rigidBodySystem.addBody(body0);

    body0->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);

    // ground
    RigidBody* ground = new RigidBody(1.0f, new Plane({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), createPlane({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}));
    ground->id = bodyId;
    ground->x = {0.0f, -1.0f, 0.0f};
    ground->fixed = true;

    int groundId = bodyId++;
    bodyMap[groundId] = ground;
    rigidBodySystem.addBody(ground);

    ground->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);

    std::cout << "Custom scenario loaded: sphere spherical joint." << std::endl;
}

void Scenarios::createCustomScenario3(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading custom scenario: sphere between two vertical prismatics with gravity." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    // Define the prismatic dimensions and positions
    Eigen::Vector3f prismaticDim(0.5f, 10.0f, 0.5f);
    Eigen::Vector3f prismatic1Position(-1.0f, 12.0f, 0.0f);
    Eigen::Vector3f prismatic2Position(1.0f, 12.0f, 0.0f);
    Eigen::Vector3f spherePosition(0.0f, 7.0f, 0.0f);

    // Create the first vertical prismatic
    RigidBody* prismatic1 = new RigidBody(1.0f, new Box(prismaticDim), createBox(prismaticDim));
    prismatic1->id = bodyId;
    prismatic1->x = prismatic1Position;
    prismatic1->fixed = true;
    prismatic1->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);

    // Create the second vertical prismatic
    RigidBody* prismatic2 = new RigidBody(1.0f, new Box(prismaticDim), createBox(prismaticDim));
    prismatic2->id = bodyId+1;
    prismatic2->x = prismatic2Position;
    prismatic2->fixed = true;
    prismatic2->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);

    // Create the sphere
    RigidBody* sphere = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sphere->id = bodyId+2;
    sphere->x = spherePosition;
    sphere->mesh->setSurfaceColor({1.0f, 5.f, 0.1f})->setTransparency(0.6f);

    int prismatic1Id = bodyId++;
    int prismatic2Id = bodyId++;
    int sphereId = bodyId++;

    bodyMap[prismatic1Id] = prismatic1;
    bodyMap[prismatic2Id] = prismatic2;
    bodyMap[sphereId] = sphere;

    rigidBodySystem.addBody(prismatic1);
    rigidBodySystem.addBody(prismatic2);
    rigidBodySystem.addBody(sphere);

    // Create prismatic joints (constraining sphere to move along the prismatics)
    Prismatic* prismaticJoint1 = new Prismatic(bodyMap[sphereId], bodyMap[prismatic1Id], Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, prismaticDim.y() / 2.0f, 0.0f), Eigen::Vector3f::UnitY());
    Prismatic* prismaticJoint2 = new Prismatic(bodyMap[sphereId], bodyMap[prismatic2Id], Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, prismaticDim.y() / 2.0f, 0.0f), Eigen::Vector3f::UnitY());
    rigidBodySystem.addJoint(prismaticJoint1);
    rigidBodySystem.addJoint(prismaticJoint2);

    std::cout << "Custom scenario loaded: sphere between two vertical prismatics with gravity." << std::endl;
}

void Scenarios::createCustomScenario4(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading spherical joint between two boxes." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    // Define the dimensions and positions
    Eigen::Vector3f boxDim1(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f boxDim2(1.0f, 1.0f, 1.0f);

    // Create the first box
    RigidBody* box1 = new RigidBody(1.0f, new Box(boxDim1), createBox(boxDim1));
    box1->id = bodyId;
    box1->x = {0.0f, 5.0f, 0.0f};

    // Create the second box
    RigidBody* sphere = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sphere->id = bodyId+1;
    sphere->x = {0.0f, 3.0f, 0.0f};
    sphere->fixed = true;

    int box1Id = bodyId++;
    int sphereId = bodyId++;

    bodyMap[box1Id] = box1;
    bodyMap[sphereId] = sphere;

    rigidBodySystem.addBody(box1);
    rigidBodySystem.addBody(sphere);

    // Create the spherical joint
    Spherical* sphericalJoint = new Spherical(bodyMap[box1Id], bodyMap[sphereId], {0.0f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f});
    rigidBodySystem.addJoint(sphericalJoint);

    std::cout << "Custom scenario loaded: spherical joint between two boxes." << std::endl;
}

void Scenarios::createCustomScenario5(RigidBodySystem &rigidBodySystem, float ball_x = 0.0f, float ball_y = 7.0f, float ball_z = 0.0f)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading custom scenario: sphere between three vertical prismatics with gravity." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    float prismaticSpacing = 1.0f;
    float prismatic = 10.0f;
    float sphereRadius = 0.5f;

    // Define the prismatic dimensions and positions
    Eigen::Vector3f prismaticDimX(prismatic, prismaticSpacing, prismaticSpacing);
    Eigen::Vector3f prismaticDimY(prismaticSpacing, prismatic, prismaticSpacing);
    Eigen::Vector3f prismaticDimZ(prismaticSpacing, prismaticSpacing, prismatic);
    Eigen::Vector3f prismatic1Position(-prismatic / 2.0f, 0.0f, 0.0f);
    Eigen::Vector3f prismatic2Position(0.0f, -prismatic / 2.0f, 0.0f);
    Eigen::Vector3f prismatic3Position(0.0f, 0.0f, -prismatic / 2.0f);
    Eigen::Vector3f spherePosition(ball_x, ball_y, ball_z);

    // Create the first vertical prismatic along x-axis
    RigidBody* prismatic1 = new RigidBody(1.0f, new Box(prismaticDimX), createBox(prismaticDimX));
    prismatic1->id = bodyId;
    prismatic1->x = prismatic1Position;
    prismatic1->fixed = true;
    prismatic1->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);

    // Create the second vertical prismatic along y-axis
    RigidBody* prismatic2 = new RigidBody(1.0f, new Box(prismaticDimY), createBox(prismaticDimY));
    prismatic2->id = bodyId+1;
    prismatic2->x = prismatic2Position;
    prismatic2->fixed = true;
    prismatic2->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);

    // Create the third vertical prismatic along z-axis
    RigidBody* prismatic3 = new RigidBody(1.0f, new Box(prismaticDimZ), createBox(prismaticDimZ));
    prismatic3->id = bodyId+2;
    prismatic3->x = prismatic3Position;
    prismatic3->fixed = true;
    prismatic3->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setSmoothShade(false)->setTransparency(0.4f);

    // Create the sphere
    RigidBody* sphere = new RigidBody(1.0f, new Sphere(sphereRadius), createSphere(sphereRadius));
    sphere->id = bodyId+3;
    sphere->x = spherePosition;
    sphere->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f})->setTransparency(0.6f);

    int prismatic1Id = bodyId++;
    int prismatic2Id = bodyId++;
    int prismatic3Id = bodyId++;
    int sphereId = bodyId++;

    bodyMap[prismatic1Id] = prismatic1;
    bodyMap[prismatic2Id] = prismatic2;
    bodyMap[prismatic3Id] = prismatic3;
    bodyMap[sphereId] = sphere;

    rigidBodySystem.addBody(prismatic1);
    rigidBodySystem.addBody(prismatic2);
    rigidBodySystem.addBody(prismatic3);
    rigidBodySystem.addBody(sphere);

    // Create prismatic joints (constraining sphere to move along the prismatics)
    // x end prismatic
    Prismatic* prismaticJoint1 = new Prismatic(bodyMap[sphereId], bodyMap[prismatic1Id], Eigen::Vector3f(-prismatic, 0.0f, 0.0f), Eigen::Vector3f(prismaticDimX.x() / 2.0f, 0.0f, 0.0f), Eigen::Vector3f::UnitX());
    // y
    Prismatic* prismaticJoint2 = new Prismatic(bodyMap[sphereId], bodyMap[prismatic2Id], Eigen::Vector3f(0.0f, -prismatic, 0.0f), Eigen::Vector3f(0.0f, prismaticDimY.y() / 2.0f, 0.0f), Eigen::Vector3f::UnitY());
    // z
    Prismatic* prismaticJoint3 = new Prismatic(bodyMap[sphereId], bodyMap[prismatic3Id], Eigen::Vector3f(0.0f, 0.0f, -prismatic), Eigen::Vector3f(0.0f, 0.0f, prismaticDimZ.z() / 2.0f), Eigen::Vector3f::UnitZ());
    rigidBodySystem.addJoint(prismaticJoint1);
    rigidBodySystem.addJoint(prismaticJoint2);
    rigidBodySystem.addJoint(prismaticJoint3);

    std::cout << "Custom scenario loaded: sphere between three vertical prismatics with gravity." << std::endl;
}

void Scenarios::createCustomScenario6(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading custom scenario: hinge joint between two boxes." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    // Define the dimensions and positions
    Eigen::Vector3f boxDim1(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f boxDim2(1.0f, 1.0f, 1.0f);

    // Create the first box
    RigidBody* box1 = new RigidBody(1.0f, new Box(boxDim1), createBox(boxDim1));
    box1->id = bodyId;
    box1->x = {0.0f, 5.0f, 0.0f};

    // Create the second box
    RigidBody* sph = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
    sph->id = bodyId+1;
    sph->x = {0.0f, 3.0f, 0.0f};
    sph->fixed = true;

    int box1Id = bodyId++;
    int sphId = bodyId++;

    bodyMap[box1Id] = box1;
    bodyMap[sphId] = sph;

    rigidBodySystem.addBody(box1);
    rigidBodySystem.addBody(sph);

    // Create the hinge joint
    Eigen::Vector3f hingeAxis(1.0f, 0.0f, 0.0f); // Hinge axis in the local coordinate system
    Hinge* hingeJoint = new Hinge(bodyMap[box1Id], bodyMap[sphId], {0.0f, -0.5f, 0.0f}, Eigen::Quaternionf::Identity(), {0.0f, 0.5f, 0.0f}, Eigen::Quaternionf::Identity(), hingeAxis);
    rigidBodySystem.addJoint(hingeJoint);

    std::cout << "Custom scenario loaded: hinge joint between two boxes." << std::endl;
}

void Scenarios::createCustomScenario7(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading enhanced tensile table scenario." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    // Dimensions
    Eigen::Vector3f platformDim(5.0f, 0.2f, 5.0f);
    Eigen::Vector3f strutDim(0.2f, 3.5f, 0.2f);
    float cableLength = 4.0f;

    // Create top platform
    RigidBody* topPlatform = new RigidBody(10.0f, new Box(platformDim), createBox(platformDim));
    topPlatform->id = bodyId;
    topPlatform->x = {0.0f, 2.0f, 0.0f};
    topPlatform->mesh->setSurfaceColor({0.8f, 0.8f, 0.8f});

    // Create bottom platform
    RigidBody* bottomPlatform = new RigidBody(10.0f, new Box(platformDim), createBox(platformDim));
    bottomPlatform->id = bodyId+1;
    bottomPlatform->x = {0.0f, -2.0f, 0.0f};
    bottomPlatform->mesh->setSurfaceColor({0.5f, 0.5f, 0.5f});

    int topPlatformId = bodyId++;
    int bottomPlatformId = bodyId++;

    bodyMap[topPlatformId] = topPlatform;
    bodyMap[bottomPlatformId] = bottomPlatform;

    rigidBodySystem.addBody(topPlatform);
    rigidBodySystem.addBody(bottomPlatform);

    // Create struts
    std::vector<RigidBody*> struts;
    std::vector<int> strutIds;
    Eigen::Vector3f strutPositions[4] = {
        {-2.0f, 0.0f, -2.0f},
        {2.0f, 0.0f, -2.0f},
        {-2.0f, 0.0f, 2.0f},
        {2.0f, 0.0f, 2.0f}};

    for (int i = 0; i < 4; ++i)
    {
        RigidBody* strut = new RigidBody(5.0f, new Box(strutDim), createBox(strutDim));
        strut->id = bodyId;
        strut->x = strutPositions[i];
        strut->fixed = true;
        strut->mesh->setSurfaceColor({0.6f, 0.3f, 0.3f});

        int strutId = bodyId++;
        strutIds.push_back(strutId);
        bodyMap[strutId] = strut;
        rigidBodySystem.addBody(strut);
        struts.push_back(strut);
    }

    // Add spherical joints connecting the struts to the platforms
    for (int i = 0; i < 4; ++i)
    {
        // Top spherical joints
        rigidBodySystem.addJoint(new Spherical(bodyMap[topPlatformId], bodyMap[strutIds[i]],
            {platformDim.x() / 2 * (i % 2 == 0 ? -1 : 1), 0.0f, platformDim.z() / 2 * (i < 2 ? -1 : 1)},
            {0.0f, strutDim.y() / 2, 0.0f}));

        // Bottom spherical joints
        rigidBodySystem.addJoint(new Spherical(bodyMap[bottomPlatformId], bodyMap[strutIds[i]],
            {platformDim.x() / 2 * (i % 2 == 0 ? -1 : 1), 0.0f, platformDim.z() / 2 * (i < 2 ? -1 : 1)},
            {0.0f, -strutDim.y() / 2, 0.0f}));
    }

    // Add distance joints (cables)
    // Cables connecting the top platform to the bottom platform via struts
    for (int i = 0; i < 4; ++i)
    {
        // Top to bottom platform cables
        rigidBodySystem.addJoint(new Distance(bodyMap[topPlatformId], bodyMap[bottomPlatformId],
            {platformDim.x() / 2 * (i % 2 == 0 ? -1 : 1), 0.0f, platformDim.z() / 2 * (i < 2 ? -1 : 1)},
            {platformDim.x() / 2 * (i % 2 == 0 ? -1 : 1), 0.0f, platformDim.z() / 2 * (i < 2 ? -1 : 1)}, cableLength));

        // Top platform to struts
        rigidBodySystem.addJoint(new Distance(bodyMap[topPlatformId], bodyMap[strutIds[i]],
            {platformDim.x() / 2 * (i % 2 == 0 ? -1 : 1), 0.0f, platformDim.z() / 2 * (i < 2 ? -1 : 1)},
            {0.0f, strutDim.y() / 2, 0.0f}, cableLength));

        // Bottom platform to struts
        rigidBodySystem.addJoint(new Distance(bodyMap[bottomPlatformId], bodyMap[strutIds[i]],
            {platformDim.x() / 2 * (i % 2 == 0 ? -1 : 1), 0.0f, platformDim.z() / 2 * (i < 2 ? -1 : 1)},
            {0.0f, -strutDim.y() / 2, 0.0f}, cableLength));
    }

    std::cout << "Enhanced tensile table scenario loaded." << std::endl;
}

void Scenarios::createCustomScenario8(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading custom scenario: multiple spheres inside a hollow box." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    // Define the box dimensions and positions of vertices
    Eigen::Vector3f boxDim(10.0f, 10.0f, 10.0f);
    Eigen::Vector3f boxCenter(0.0f, 5.0f, 0.0f);

    std::vector<Eigen::Vector3f> vertices = {
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, -boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, -boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, -boxDim.y() / 2, boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, -boxDim.y() / 2, boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, boxDim.y() / 2, boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, boxDim.y() / 2, boxDim.z() / 2)};

    // Create rigid bodies for each vertex
    std::vector<RigidBody*> boxVertices;
    std::vector<int> vertexIds;

    for (const auto &vertex : vertices)
    {
        RigidBody* vertexBody = new RigidBody(1.0f, new Sphere(0.2f), createSphere(0.2f));
        vertexBody->id = bodyId;
        vertexBody->x = vertex;
        vertexBody->fixed = true;
        vertexBody->mesh->setSurfaceColor({0.6f, 0.6f, 0.6f})->setTransparency(0.6f);

        int vertexId = bodyId++;
        vertexIds.push_back(vertexId);
        bodyMap[vertexId] = vertexBody;
        rigidBodySystem.addBody(vertexBody);
        boxVertices.push_back(vertexBody);
    }

    // Add distance joints between each pair of vertices to form the edges of the box
    std::vector<std::pair<int, int>> edges = {
        {0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3}, {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}};

    for (const auto &edge : edges)
    {
        rigidBodySystem.addJoint(new Distance(bodyMap[vertexIds[edge.first]], bodyMap[vertexIds[edge.second]],
            {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
            (boxVertices[edge.first]->x - boxVertices[edge.second]->x).norm()));
    }

    // Define the sphere radius and create multiple spheres inside the box
    float sphereRadius = 0.5f;
    int numSpheres = 10;

    std::vector<RigidBody*> spheres;
    std::vector<int> sphereIds;

    for (int i = 0; i < numSpheres; ++i)
    {
        RigidBody* sphere = new RigidBody(1.0f, new Sphere(sphereRadius), createSphere(sphereRadius));
        sphere->id = bodyId;
        sphere->x = boxCenter + Eigen::Vector3f(
                                    static_cast<float>(rand()) / RAND_MAX * boxDim.x() - boxDim.x() / 2,
                                    static_cast<float>(rand()) / RAND_MAX * boxDim.y() - boxDim.y() / 2,
                                    static_cast<float>(rand()) / RAND_MAX * boxDim.z() - boxDim.z() / 2);
        sphere->mesh->setSurfaceColor({1.0f, 0.1f, 0.1f})->setTransparency(0.6f);

        int sphereId = bodyId++;
        sphereIds.push_back(sphereId);
        bodyMap[sphereId] = sphere;
        rigidBodySystem.addBody(sphere);
        spheres.push_back(sphere);
    }

    // Add distance joints to constrain spheres within the box
    for (int i = 0; i < numSpheres; ++i)
    {
        for (int j = 0; j < vertexIds.size(); ++j)
        {
            rigidBodySystem.addJoint(new Distance(bodyMap[sphereIds[i]], bodyMap[vertexIds[j]],
                {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
                (spheres[i]->x - boxVertices[j]->x).norm()));
        }
    }

    std::cout << "Custom scenario loaded: multiple spheres inside a hollow box." << std::endl;
}

void Scenarios::createCustomScenario9(RigidBodySystem &rigidBodySystem)
{
    rigidBodySystem.clear();
    polyscope::removeAllStructures();

    std::cout << "Loading custom scenario: multiple spheres inside a hollow box." << std::endl;

    // Map to store body IDs
    std::map<int, RigidBody*> bodyMap;
    int bodyId = 0;

    // Define the box dimensions and positions of vertices
    Eigen::Vector3f boxDim(10.0f, 10.0f, 10.0f);
    Eigen::Vector3f boxCenter(0.0f, 5.0f, 0.0f);

    std::vector<Eigen::Vector3f> vertices = {
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, -boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, -boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, boxDim.y() / 2, -boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, -boxDim.y() / 2, boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, -boxDim.y() / 2, boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(-boxDim.x() / 2, boxDim.y() / 2, boxDim.z() / 2),
        boxCenter + Eigen::Vector3f(boxDim.x() / 2, boxDim.y() / 2, boxDim.z() / 2) };

    // Create rigid bodies for each vertex using Box
    std::vector<RigidBody*> boxVertices;
    std::vector<int> vertexIds;
    for (const auto& vertex : vertices)
    {
        RigidBody* vertexBody = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.2f, 0.2f, 0.2f)), createBox(Eigen::Vector3f(0.2f, 0.2f, 0.2f)));
        vertexBody->id = bodyId;
        vertexBody->x = vertex;
        vertexBody->fixed = true;
        vertexBody->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setTransparency(0.6f);

        int vertexId = bodyId++;
        vertexIds.push_back(vertexId);
        bodyMap[vertexId] = vertexBody;
        rigidBodySystem.addBody(vertexBody);
        boxVertices.push_back(vertexBody);
    }

    // Add rigid bodies for each edge of the box
    std::vector<std::pair<int, int>> edges = {
        {0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3}, {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7} };

    for (const auto& edge : edges)
    {
        Eigen::Vector3f startPos = boxVertices[edge.first]->x;
        Eigen::Vector3f endPos = boxVertices[edge.second]->x;
        Eigen::Vector3f edgeCenter = (startPos + endPos) / 2;
        Eigen::Vector3f edgeDim = (endPos - startPos).cwiseAbs() + Eigen::Vector3f(0.2f, 0.2f, 0.2f); // Add some thickness

        RigidBody* edgeBody = new RigidBody(1.0f, new Box(edgeDim), createBox(edgeDim));
        edgeBody->id = bodyId;
        edgeBody->x = edgeCenter;
        edgeBody->fixed = true;
        edgeBody->mesh->setSurfaceColor({ 0.3f, 0.3f, 0.3f })->setTransparency(0.6f);

        int edgeId = bodyId++;
        bodyMap[edgeId] = edgeBody;
        rigidBodySystem.addBody(edgeBody);
    }

    // Add rigid bodies for each face of the box
    std::vector<std::tuple<int, int, int, int>> faces = {
        {0, 1, 3, 2}, {0, 1, 5, 4}, {0, 2, 6, 4}, {1, 3, 7, 5}, {2, 3, 7, 6}, {4, 5, 7, 6}
    };

    for (const auto& face : faces)
    {
        Eigen::Vector3f v0 = boxVertices[std::get<0>(face)]->x;
        Eigen::Vector3f v1 = boxVertices[std::get<1>(face)]->x;
        Eigen::Vector3f v2 = boxVertices[std::get<2>(face)]->x;
        Eigen::Vector3f v3 = boxVertices[std::get<3>(face)]->x;

        Eigen::Vector3f faceCenter = (v0 + v1 + v2 + v3) / 4;
        Eigen::Vector3f faceDim = (v1 - v0).cwiseAbs() + (v3 - v0).cwiseAbs() + Eigen::Vector3f(0.2f, 0.2f, 0.2f); // Add some thickness

        RigidBody* faceBody = new RigidBody(1.0f, new Box(faceDim), createBox(faceDim));
        faceBody->id = bodyId;
        faceBody->x = faceCenter;
        faceBody->fixed = true;
        faceBody->mesh->setSurfaceColor({ 0.1f, 0.1f, 0.1f })->setTransparency(0.6f);

        int faceId = bodyId++;
        bodyMap[faceId] = faceBody;
        rigidBodySystem.addBody(faceBody);
    }

    // Define the sphere radius and create multiple spheres inside the box
    float sphereRadius = 0.5f;
    int numSpheres = 20;

    std::vector<RigidBody*> spheres;
    std::vector<int> sphereIds;

    for (int i = 0; i < numSpheres; ++i)
    {
        RigidBody* sphere = new RigidBody(1.0f, new Sphere(sphereRadius), createSphere(sphereRadius));
        sphere->id = bodyId;
        sphere->x = boxCenter + Eigen::Vector3f(
            static_cast<float>(rand()) / RAND_MAX * boxDim.x() - boxDim.x() / 2,
            static_cast<float>(rand()) / RAND_MAX * boxDim.y() - boxDim.y() / 2,
            static_cast<float>(rand()) / RAND_MAX * boxDim.z() - boxDim.z() / 2);
        sphere->xdot = Eigen::Vector3f::Random() * 2.f;
        sphere->omega = Eigen::Vector3f::Random();
        sphere->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f })->setTransparency(0.6f);

        int sphereId = bodyId++;
        sphereIds.push_back(sphereId);
        bodyMap[sphereId] = sphere;
        rigidBodySystem.addBody(sphere);
        spheres.push_back(sphere);
    }

    std::cout << "Custom scenario loaded: multiple spheres inside a hollow box." << std::endl;
}
