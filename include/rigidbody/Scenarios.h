#pragma once

#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "joint/Spherical.h"
#include "joint/Hinge.h"
#include "util/Types.h"
#include "util/MeshAssets.h"

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

        const float radius = 0.5f;
        // Create two layers of "marbles", in a grid layout.
        //
        for (int i = 0; i < 9; ++i)
        {
            for (int j = 0; j < 9; ++j)
            {
                RigidBody* body1 = new RigidBody(1.0f, new Sphere(radius), createSphere(radius));
                body1->x.x() = -4.0f + (float)i * 1.0f;
                body1->x.z() = -4.0f + (float)j * 1.0f;
                body1->x.y() = 2.0f;
                rigidBodySystem.addBody(body1);
                body1->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                body1->mesh->setTransparency(0.8f);
                RigidBody* body2 = new RigidBody(1.0f, new Sphere(radius), createSphere(radius));
                body2->x.x() = -4.0f + (float)i * 1.0f;
                body2->x.z() = -4.0f + (float)j * 1.0f;
                body2->x.y() = 3.0f;
                rigidBodySystem.addBody(body2);
                body2->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                body2->mesh->setTransparency(0.8f);
                RigidBody* body3 = new RigidBody(1.0f, new Sphere(radius), createSphere(radius));
                body3->x.x() = -4.0f + (float)i * 1.0f;
                body3->x.z() = -4.0f + (float)j * 1.0f;
                body3->x.y() = 4.0f;
                rigidBodySystem.addBody(body3);
                body3->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
                body3->mesh->setTransparency(0.8f);
            }
        }

        // Create big marble
        RigidBody* big = new RigidBody(10000.0f, new Sphere(4.0f), createSphere(4.0f));
        big->x.x() = 0;
        big->x.z() = 0;
        big->x.y() = 9.0f;
        rigidBodySystem.addBody(big);
        big->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f });
        big->mesh->setTransparency(0.8f);

        // Create the box to hold the marbles.
        const Eigen::Vector3f sideDim(Eigen::Vector3f(0.4f, 4.0f, 10.0f));
        const Eigen::Vector3f botDim(Eigen::Vector3f(10.0f, 0.4f, 10.0f));
        RigidBody* body0 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim) );
        RigidBody* body1 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim));
        RigidBody* body2 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim));
        RigidBody* body3 = new RigidBody(1.0f, new Box(sideDim), createBox(sideDim));
        RigidBody* body4 = new RigidBody(1.0f, new Box(botDim), createBox(botDim));
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
        const float radius = 0.5f;
        RigidBody* bodySphere = new RigidBody(1.0f, new Sphere(radius), createSphere(radius));
        bodySphere->x.y() = 4.0f;
        bodySphere->omega = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
        bodySphere->mesh->setTransparency(0.8f);

        // Create a box that will act as the ground.
        const Eigen::Vector3f dim(10.0f, 0.4f, 10.0f);
        RigidBody* bodyBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
        bodyBox->fixed = true;

        rigidBodySystem.addBody(bodySphere);
        rigidBodySystem.addBody(bodyBox);

        bodySphere->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(0.0f);
        bodyBox->mesh->setSurfaceColor({ 0.2f, 0.2f, 0.2f })->setSmoothShade(false)->setTransparency(0.4f);
    }

    // Stack of boxes
    //
    static void createStack(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading stack scenario." << std::endl;

        // Create a box that will act as the ground.
        RigidBody* bodyBox = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 1.0f, 10.0f)), createBox(Eigen::Vector3f(10, 1.0f, 10)));
        bodyBox->fixed = true;
        bodyBox->mesh->setSurfaceColor({ 0.2f, 0.2f, 0.2f })->setSmoothShade(false)->setTransparency(0.4f);

        rigidBodySystem.addBody(bodyBox);

        const int N = 10;
        for (int i = 1; i <= N; i++)
        {
            RigidBody* body1 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
            body1->x = { -4.0f, 1.5f * i - 0.5f, -4.0f };
            rigidBodySystem.addBody(body1);
            body1->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(0.0f);
            body1->mesh->setTransparency(0.8f);
            body1->mesh->setSmoothShade(true);

            RigidBody* body2 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
            body2->x = { 4.0f, 1.5f * i - 0.5f, -4.0f };
            rigidBodySystem.addBody(body2);
            body2->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(0.0f);
            body2->mesh->setTransparency(0.8f);
            body2->mesh->setSmoothShade(true);

            RigidBody* body3 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
            body3->x = { -4.0f, 1.5f * i - 0.5f, 4.0f };
            rigidBodySystem.addBody(body3);
            body3->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(0.0f);
            body3->mesh->setTransparency(0.8f);
            body3->mesh->setSmoothShade(true);

            RigidBody* body4 = new RigidBody(1.0f, new Sphere(0.5f), createSphere(0.5f));
            body4->x = { 4.0f, 1.5f * i - 0.5f, 4.0f };
            rigidBodySystem.addBody(body4);
            body4->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(0.0f);
            body4->mesh->setTransparency(0.8f);
            body4->mesh->setSmoothShade(true);

            if (i < N)
            {
                RigidBody* body5 = new RigidBody(1.0f, new Box(Eigen::Vector3f(10.0f, 0.5f, 10.0f)), createBox(Eigen::Vector3f(10, 0.5f, 10)));
                body5->x = { 0, 1.5f * i + 0.25f, 0 };
                rigidBodySystem.addBody(body5);
                body5->mesh->setSurfaceColor({ 1.0f, 0.1f, 0.1f })->setEdgeWidth(0.0f);
                body5->mesh->setTransparency(0.8f);
            }
        }

        RigidBody* topBox = new RigidBody(20000.0f, new Box(Eigen::Vector3f(15.0f, 1.0f, 15.0f)), createBox(Eigen::Vector3f(15.0f, 1.0f, 15.0f)));
        topBox->x = { 0, 1.5f * N + 0.5f, 0 };
        rigidBodySystem.addBody(topBox);
        topBox->mesh->setSurfaceColor({ 0.1f, 0.2f, 1.0f })->setEdgeWidth(0.0f);
        topBox->mesh->setTransparency(0.8f);
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
        Eigen::Vector3f dim({ 0.25f, 1.5f, 0.25f });
        RigidBody* topBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
        topBox->x = { 0.0f, 1.5f*(float)N + 3.0f, 0.0f };
        topBox->fixed = true;
        rigidBodySystem.addBody(topBox);

        topBox->mesh->setSurfaceColor({ 1.0f, 1.0f, 0.1f });
        topBox->mesh->setEdgeWidth(0.0f);

        const Eigen::Vector3f dx(0.0f, 1.5f, 0.0f);
        RigidBody* parent = topBox;
        for (int i = 0; i < N-1; ++i)
        {
            // Create the next box in the chain.
            RigidBody* nextBox = nullptr;
            Joint* j = nullptr;
            if (i == (N - 2))
            {
                dim = { 4.0f, 4.0f, 4.0f };
                nextBox = new RigidBody(1000.0f, new Box(dim), createBox(dim));
                nextBox->mesh->setSurfaceColor({ 1.0f, 0.2f, 0.2f });
                nextBox->mesh->setEdgeWidth(0.0f);
                const Eigen::Vector3f extraDx({ 0.0f, 2.75f, 0.0f });
                nextBox->x = parent->x - extraDx;

                // Add a hinge between large box and parent
                j = new Hinge(parent, nextBox, -0.5f * extraDx, Eigen::Quaternionf::Identity(), 0.5f * extraDx, Eigen::Quaternionf::Identity());
            }
            else
            {
                nextBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
                nextBox->mesh->setSurfaceColor({ 0.1f, 0.2f, 1.0f });
                nextBox->mesh->setEdgeWidth(0.0f);
                nextBox->x = parent->x - dx;

                // Add a hinge between parent->nextBox
                j = new Hinge(parent, nextBox, -0.5f * dx, Eigen::Quaternionf::Identity(), 0.5f * dx, Eigen::Quaternionf::Identity());
            }

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
        const float height = 2.0f;
        const float radius = 1.0f;
        RigidBody* cyl = new RigidBody(1.0f, new Cylinder(height, radius), createCylinder(16, radius, height));
        cyl->x = { 0.0f, 2.0f, 0.0f };
        cyl->q = Eigen::AngleAxisf(0.57f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

        // Create a ground plane.
        RigidBody* plane = new RigidBody(1.0f, new Plane({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), "");
        plane->x = { 0.0f, 0.0f, 0.0f };
        plane->fixed = true;

        rigidBodySystem.addBody(cyl);
        rigidBodySystem.addBody(plane);
    }

    // Simple 4 wheeled car + hinges + box chassis
    //
    static void createCarScene(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading car scenario." << std::endl;

        // Create a car.
        RigidBody* chassis = new RigidBody(5.0f, new Box({ 2.0f, 0.5f, 3.0f }), createBox({ 2.0f, 0.5f, 3.0f }) );
        RigidBody* lfwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));
        RigidBody* rfwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));
        RigidBody* lrwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));
        RigidBody* rrwheel = new RigidBody(1.0f, new Cylinder(0.2f, 0.5f), createCylinder(16, 0.5f, 0.2f));
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

    // "Rope" bridge
    static void createRopeBridgeScene(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        polyscope::removeAllStructures();

        std::cout << "Loading bridge scenario." << std::endl;

        const int N = 20;

        const float dx = 0.6f;
        const float y = 3.0f;
        const float x0 = -(N / 2) * dx;
        float x = x0;
        // Create a box.
        const Eigen::Vector3f dim({ 0.5f, 0.1f, 1.0f });
        RigidBody* firstBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
        firstBox->x = { x, y, 0.0f };
        firstBox->fixed = true;
        rigidBodySystem.addBody(firstBox);
        firstBox->mesh->setSurfaceColor({ 1.0f, 1.0f, 0.1f })->setEdgeWidth(0.0f);

        RigidBody* parent = firstBox;
        for (int i = 0; i < N - 1; ++i)
        {
            // Create the next box in the chain.
            x += dx;
            RigidBody* nextBox = new RigidBody(1.0f, new Box(dim), createBox(dim));
            nextBox->x = { x, y, 0.0f };
            nextBox->mesh->setSurfaceColor({ 0.1f, 0.2f, 1.0f })->setEdgeWidth(0.0f);

            // Add new box 
            rigidBodySystem.addBody(nextBox);

            // Add spherical joints between parent->nextBox
            Joint* j0 = new Spherical(parent, nextBox, { dx / 2.0f, 0.0f, 0.5f }, { -dx / 2.0f, 0.0f, 0.5f });
            Joint* j1 = new Spherical(parent, nextBox, { dx / 2.0f, 0.0f, -0.5f }, { -dx / 2.0f, 0.0f, -0.5f });

            // Add spherical joints to the rigid body system.
            rigidBodySystem.addJoint(j0);
            rigidBodySystem.addJoint(j1);
            parent = nextBox;
        }
        parent->fixed = true;
        parent->mesh->setSurfaceColor({ 1.0f, 1.0f, 0.1f });


        // Create a sphere.
        const float radius = 0.5f;
        RigidBody* bodySphere = new RigidBody(1000.0f, new Sphere(radius), createSphere(radius));
        bodySphere->x = { x0, y + 1.0f, 0.0f };
        bodySphere->omega = { 0.0f, 0.0f, -5.0f };
        bodySphere->xdot = { 1.0f, 0.0f, 0.0f };
        bodySphere->mesh->setSurfaceColor({ 0.1f, 1.0f, 0.2f })->setEdgeWidth(0.0f)->setTransparency(0.8f);
        bodySphere->mesh->setSmoothShade(true);

        rigidBodySystem.addBody(bodySphere);

    }

};
