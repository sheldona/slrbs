#pragma once

#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "joint/Spherical.h"
#include "joint/Hinge.h"
#include "util/Types.h"

#include <cstdlib>
#include <Eigen/Dense>

class Scenarios
{
public:
    // Box filled with balls.
    //
    static void createMarbleBox(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading marble box scenario" << std::endl;

        // Create two layers of "marbles", in a grid layout.
        //
        for (int i = 0; i < 9; ++i)
        {
            for (int j = 0; j < 9; ++j)
            {
                RigidBody* body1 = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
                body1->x.x() = -4.0f + (float)i * 1.0f;
                body1->x.z() = -4.0f + (float)j * 1.0f;
                body1->x.y() = 2.0f;
                rigidBodySystem.addBody(body1);
                body1->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                body1->mesh->setTransparency(0.8f);
                RigidBody* body2 = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
                body2->x.x() = -4.0f + (float)i * 1.0f;
                body2->x.z() = -4.0f + (float)j * 1.0f;
                body2->x.y() = 3.0f;
                rigidBodySystem.addBody(body2);
                body2->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                body2->mesh->setTransparency(0.8f);
            }
        }

        // Create the box to hold the marbles.
        RigidBody* body0 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body2 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.0f)), "resources/box_side.obj");
        RigidBody* body3 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.4f, 4.0f, 10.4f)), "resources/box_side.obj");
        RigidBody* body4 = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "resources/box_bot.obj");
        body0->fixed = true;
        body1->fixed = true;
        body2->fixed = true;
        body3->fixed = true;
        body4->fixed = true;
        body0->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body1->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body2->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body3->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body4->mesh->setSurfaceColor({ 0.6f, 0.6f, 0.6f })->setSmoothShade(false)->setTransparency(0.4f);
        body0->x = { 4.75f, 2.0f, 0.0f };
        body1->x = { -4.75f, 2.0f, 0.0f };
        body2->x = { 0.0f, 2.0f, 4.75f };
        body2->q = Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 1, 0));
        body3->x = { 0.0f, 2.0f, -4.75f };
        body3->q = Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 1, 0));
        body4->x = { 0.0f, 0.0f, 0.0f };

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
        rigidBodySystem.addBody(body4);
    }

    // Simple sphere falling on a box.
    //
    static void createSphereOnBox(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading sphere-on-box scenario." << std::endl;

        // Create a sphere.
        RigidBody* bodySphere = new RigidBody(1.0f, new Sphere(0.5f), "resources/sphere.obj");
        bodySphere->x.y() = 4.0f;
        bodySphere->omega = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
        bodySphere->mesh->setTransparency(0.8f);

        // Create a box that will act as the ground.
        RigidBody* bodyBox = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.4f, 10.0f)), "resources/box_bot.obj");
        bodyBox->fixed = true;

        rigidBodySystem.addBody(bodySphere);
        rigidBodySystem.addBody(bodyBox);

        bodySphere->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(1.0f);
        bodyBox->mesh->setSurfaceColor({ 0.2f, 0.2f, 0.2f })->setSmoothShade(false)->setTransparency(0.4f);
    }

    // Box hanging from a box
    //
    static void createSwingingBoxes(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading swinging boxes scenario." << std::endl;

        const int N = 20;

        // Create a box.
        RigidBody* topBox = new RigidBody(1.0f, new Box({ 1.0f, 1.0f, 1.0f }), "resources/box.obj");
        topBox->x = { 0.0f, 1.5f*(float)N, 0.0f };
        topBox->fixed = true;
        rigidBodySystem.addBody(topBox);

        RigidBody* parent = topBox;
        for (int i = 0; i < N-1; ++i)
        {
            // Create the next box in the chain.
            RigidBody* nextBox = nullptr;
            if( i == (N-2) ) nextBox = new RigidBody(10000.0f, new Box({ 1.0f, 1.0f, 1.0f }), "resources/box.obj");
            else nextBox = new RigidBody(1.0f, new Box({ 1.0f, 1.0f, 1.0f }), "resources/box.obj");
            nextBox->x = parent->x - Eigen::Vector3f(0.0f, 1.5f, 0.0f);

            // Add a hinge between parent->nextBox
            Joint* j = new Hinge(parent, nextBox, { 0.0f, 0.0f, 0.0f }, Eigen::Quaternionf::Identity(), { 0.0f, 1.5f, 0.0f }, Eigen::Quaternionf::Identity());

            // Add new box and hinge to the rigid body system.
            rigidBodySystem.addBody(nextBox);
            rigidBodySystem.addJoint(j);
            parent = nextBox;
        }

        parent->xdot = { 0.0f, 0.0, 10.0f };
    }


    // Cylinder-plane collision test
    //
    static void createCylinderOnPlane(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading cylinder on plane scenario." << std::endl;

        // Create a cylinder.
        RigidBody* cyl = new RigidBody(1.0f, new Cylinder(2.0f, 1.0f), "resources/cylinder.obj");
        cyl->x = { 0.0f, 2.0f, 0.0f };
        cyl->q = Eigen::AngleAxisf(0.57f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

        // Create a ground plane.
        RigidBody* plane = new RigidBody(1.0f, new Plane({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), "");
        plane->x = { 0.0f, 0.0f, 0.0f };
        plane->fixed = true;

        rigidBodySystem.addBody(cyl);
        rigidBodySystem.addBody(plane);
    }

    // Box hanging from a box
    //
    static void createCarScene(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading car scenario." << std::endl;

        // Create a car.
        RigidBody* chassis = new RigidBody(5.0f, new Box({ 1.0f, 0.5f, 1.5f }), "resources/chassis.obj");
        RigidBody* lfwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), "resources/wheel.obj");
        RigidBody* rfwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), "resources/wheel.obj");
        RigidBody* lrwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), "resources/wheel.obj");
        RigidBody* rrwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), "resources/wheel.obj");
        chassis->x = { 0.0f, 0.5f, 0.0f };
        chassis->xdot = { 0.0f, 0.0f, -10.0f };
        lfwheel->x = { 1.0f, 0.5f, 1.5f };
        rfwheel->x = { -1.0f, 0.5f, 1.5f };
        lrwheel->x = { 1.0f, 0.5f, -1.5f };
        rrwheel->x = { -1.0f, 0.5f, -1.5f };
        lfwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        rfwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        lrwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        rrwheel->q = Eigen::AngleAxisf(1.57079f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  
        // Create a ground plane.
        RigidBody* plane = new RigidBody(1.0f, new Plane({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }), "");
        plane->x = { 0.0f, 0.0f, 0.0f };
        plane->fixed = true;

        // Setup the vehicle joints/constraints.
        Hinge* lfhinge = new Hinge(chassis, lfwheel,
            { 1.0f, 0.0f, 1.5f },
            Eigen::Quaternionf::Identity(),
            { 0.0f, 0.0f, 0.0f },
            Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
        );

        Hinge* rfhinge = new Hinge(chassis, rfwheel,
            { -1.0f, 0.0f, 1.5f },
            Eigen::Quaternionf::Identity(),
            { 0.0f, 0.0f, 0.0f },
            Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
        );

        Hinge* lrhinge = new Hinge(chassis, lrwheel,
            { 1.0f, 0.0f, -1.5f },
            Eigen::Quaternionf::Identity(),
            { 0.0f, 0.0f, 0.0f },
            Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
        );

        Hinge* rrhinge = new Hinge(chassis, rrwheel,
            { -1.0f, 0.0f, -1.5f },
            Eigen::Quaternionf::Identity(),
            { 0.0f, 0.0f, 0.0f },
            Eigen::Quaternionf(Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 0, 1)))
        );

        rigidBodySystem.addBody(chassis);
        rigidBodySystem.addBody(lfwheel);
        rigidBodySystem.addBody(rfwheel);
        rigidBodySystem.addBody(lrwheel);
        rigidBodySystem.addBody(rrwheel);
        rigidBodySystem.addBody(plane);

        rigidBodySystem.addJoint(lfhinge);
        rigidBodySystem.addJoint(rfhinge);
        rigidBodySystem.addJoint(lrhinge);
        rigidBodySystem.addJoint(rrhinge);
    }

};
