#pragma once

#include <vector>
#include <string>
#include <functional>
#include "util/Types.h"
#include "util/MeshAssets.h"
#include "rigidbody/Scenarios.h"
#include "rigidbody/RigidBody.h"
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

class RigidBodySystem;

class Scenarios {
public:
    enum ScenarioID {
        MARBLE_BOX = 0,
        SPHERE_ON_BOX,
        SWINGING_BOXES,
        CYLINDER_ON_PLANE,
        CAR_SCENE,
        STACK,
        ROPE_BRIDGE,
        SPHERE_SPHERE_DISTANCE,
        SPHERE_INSIDE_BOX,
        BOX_ON_PLANE,
        CYLINDER_SPHERE_TEST,
        ROPE_LADDER,
        CUSTOM_DOUBLE_PENDULUM,
        CUSTOM_SPHERICAL_JOINT,
        CUSTOM_TWO_PRISMATICS,
        CUSTOM_BOX_SPHERE_JOINT,
        CUSTOM_THREE_PRISMATICS,
        CUSTOM_HINGE_JOINT,
        CUSTOM_TENSILE_TABLE,
        CUSTOM_HOLLOW_BOX,
        CUSTOM_BOX_WITH_FACES
    };

    struct ScenarioInfo {
        ScenarioID id;
        std::string name;
        std::string description;
        std::string thumbnailPath;
        std::function<void(RigidBodySystem&)> createFunction;
    };

    // Fetch metadata for UI lists, etc.
    static std::vector<ScenarioInfo> getAvailableScenarios();

    // Build the chosen scenario into your RigidBodySystem
    static bool createScenario(RigidBodySystem& system, ScenarioID id);

    // Get name/description by ID
    static ScenarioInfo getScenarioInfo(ScenarioID id);

    // All of these are defined in Scenarios.cpp
    static void createMarbleBox(RigidBodySystem&);
    static void createSphereOnBox(RigidBodySystem&);
    static void createSwingingBoxes(RigidBodySystem&);
    static void createCylinderOnPlane(RigidBodySystem&);
    static void createCarScene(RigidBodySystem&);
    static void createStack(RigidBodySystem&);
    static void createRopeBridgeScene(RigidBodySystem&);
    static void createSphereSphereDistance(RigidBodySystem&);
    static void createSphereInsideBox(RigidBodySystem&);
    static void createBoxOnPlane(RigidBodySystem&);
    static void createCylinderSphereTest(RigidBodySystem&, const Eigen::AngleAxisf&);
    static void createRopeLadder(RigidBodySystem&);
    static void createCustomScenario(RigidBodySystem&);
    static void createCustomScenario2(RigidBodySystem&);
    static void createCustomScenario3(RigidBodySystem&);
    static void createCustomScenario4(RigidBodySystem&);
    static void createCustomScenario5(RigidBodySystem&, float, float, float);
    static void createCustomScenario6(RigidBodySystem&);
    static void createCustomScenario7(RigidBodySystem&);
    static void createCustomScenario8(RigidBodySystem&);
    static void createCustomScenario9(RigidBodySystem&);
};
