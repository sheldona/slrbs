#include "util/ScenarioLoader.h"
#include "collision/Geometry.h"
#include "rigidbody/RigidBody.h"
#include "joint/Hinge.h"
#include "joint/Spherical.h"
#include "util/MeshAssets.h"
#include "util/VisualProperties.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>

// JSON parser library
#include <nlohmann/json.hpp>
#include <algorithm>
using json = nlohmann::json;


std::string ScenarioLoader::getScenariosPath() {
    // Try common paths
    std::vector<std::string> paths = {
        "./resources/scenarios/",
        "../resources/scenarios/",
        "../../resources/scenarios/",
        "./",
        "../"
    };

    for (const auto &path : paths) {
        if (std::filesystem::exists(path)) {
            return path;
        }
    }

    // Default to current directory
    return "./";
}

std::vector<std::string> ScenarioLoader::listAvailableScenarios() {
    std::vector<std::string> scenarios;

    try {
        std::string scenariosPath = getScenariosPath();
        std::cout << "Searching for scenarios in: " << scenariosPath << std::endl;

        // List all JSON files in scenarios directory
        for (const auto &entry : std::filesystem::directory_iterator(scenariosPath)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                scenarios.push_back(entry.path().filename().string());
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Error listing scenarios: " << e.what() << std::endl;
    }

    return scenarios;
}

bool ScenarioLoader::loadFromFile(RigidBodySystem &system, const std::string &filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::string fullPath = getScenariosPath() + filename;
            std::cout << "Trying to open scenario from: " << fullPath << std::endl;
            file.open(fullPath);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return false;
            }
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parseScenario(system, buffer.str());
    } catch (const std::exception &e) {
        std::cerr << "Error loading scenario from JSON: " << e.what() << std::endl;
        return false;
    }
}

bool ScenarioLoader::parseScenario(RigidBodySystem &system, const std::string &jsonContent) {
    try {
        json scenario = json::parse(jsonContent);
        std::cout << "Loading scenario: " << scenario.at("name").get<std::string>() << std::endl;

        // Clear system and visualization
        system.clear();
        g_visualProperties.clear();

        std::map<int, RigidBody*> bodyMap;
        int bodyIndex = 0;

        // Load bodies
        for (const auto &bodyJson : scenario.at("bodies")) {
            RigidBody *body = nullptr;
            Geometry *geometry = nullptr;

            // Create geometry
            if (bodyJson.count("geometry") > 0) {
                const auto &geo = bodyJson["geometry"];
                std::string type = geo.at("type").get<std::string>();
                if (type == "sphere") {
                    float radius = geo.at("radius").get<float>();
                    geometry = new Sphere(radius);
                    body = new RigidBody(bodyJson.at("mass").get<float>(), geometry, createSphere(radius));
                } else if (type == "box") {
                    Eigen::Vector3f dim(
                        geo.at("dimensions")["x"].get<float>(),
                        geo.at("dimensions")["y"].get<float>(),
                        geo.at("dimensions")["z"].get<float>());
                    geometry = new Box(dim);
                    body = new RigidBody(bodyJson.at("mass").get<float>(), geometry, createBox(dim));
                } else if (type == "cylinder") {
                    float height = geo.at("height").get<float>();
                    float radius = geo.at("radius").get<float>();
                    geometry = new Cylinder(height, radius);
                    body = new RigidBody(bodyJson.at("mass").get<float>(), geometry, createCylinder(16, radius, height));
                } else if (type == "plane") {
                    Eigen::Vector3f point(
                        geo.at("point")["x"].get<float>(),
                        geo.at("point")["y"].get<float>(),
                        geo.at("point")["z"].get<float>());
                    Eigen::Vector3f normal(
                        geo.at("normal")["x"].get<float>(),
                        geo.at("normal")["y"].get<float>(),
                        geo.at("normal")["z"].get<float>());
                    geometry = new Plane(point, normal);
                    body = new RigidBody(bodyJson.at("mass").get<float>(), geometry, "");
                }
            }

            if (!body) {
                std::cerr << "Failed to create body from JSON" << std::endl;
                continue;
            }

            // Set body properties
            body->fixed = bodyJson.value("fixed", false);
            body->x = Eigen::Vector3f(
                bodyJson.at("position")["x"].get<float>(),
                bodyJson.at("position")["y"].get<float>(),
                bodyJson.at("position")["z"].get<float>());
            body->q = Eigen::Quaternionf(
                bodyJson.at("orientation")["w"].get<float>(),
                bodyJson.at("orientation")["x"].get<float>(),
                bodyJson.at("orientation")["y"].get<float>(),
                bodyJson.at("orientation")["z"].get<float>());

            if (bodyJson.count("linear_velocity") > 0) {
                body->xdot = Eigen::Vector3f(
                    bodyJson["linear_velocity"]["x"].get<float>(),
                    bodyJson["linear_velocity"]["y"].get<float>(),
                    bodyJson["linear_velocity"]["z"].get<float>());
            }
            if (bodyJson.count("angular_velocity") > 0) {
                body->omega = Eigen::Vector3f(
                    bodyJson["angular_velocity"]["x"].get<float>(),
                    bodyJson["angular_velocity"]["y"].get<float>(),
                    bodyJson["angular_velocity"]["z"].get<float>());
            }

            // Set material properties if available
            if (bodyJson.count("material") > 0) {
                const auto &material = bodyJson["material"];
                if (material.count("restitution") > 0) {
                    body->restitution = material["restitution"].get<float>();
                }
                if (material.count("friction") > 0) {
                    body->friction = material["friction"].get<float>();
                }
                if (material.count("density") > 0) {
                    body->density = material["density"].get<float>();
                }
            }

            // Visual properties
            if (bodyJson.count("visual") > 0 && body->mesh) {
                const auto &visual = bodyJson["visual"];

                float r = 0.5f, g = 0.5f, b = 0.5f;
                float transparency = visual.value("transparency", 0.0f);
                bool smoothShade = visual.value("smooth_shade", false);
                float edgeWidth = visual.value("edge_width", 1.0f);

                // Default visualization and debug
                bool showWireframe = false, showNormals = false, showBoundingBox = false;
                float normalLength = 0.1f;
                bool showContactPoints = false, showInertiaEllipsoid = false;
                float shininess = 0.0f, reflectivity = 0.0f;
                bool showTextureSpace = false;
                std::string textureParamName = "uv_coords";
                std::optional<std::string> texturePath;
                float debugColorR = 1.0f, debugColorG = 1.0f, debugColorB = 0.0f;

                if (visual.count("color") > 0) {
                    const auto &col = visual["color"];
                    r = col.value("r", r);
                    g = col.value("g", g);
                    b = col.value("b", b);
                }

                if (visual.count("material") > 0) {
                    const auto &mat = visual["material"];
                    shininess = mat.value("shininess", shininess);
                    reflectivity = mat.value("reflectivity", reflectivity);
                }

                if (visual.count("texture") > 0) {
                    const auto &tex = visual["texture"];
                    showTextureSpace = tex.value("show_texture_space", showTextureSpace);
                    if (tex.count("param_name") > 0) textureParamName = tex["param_name"].get<std::string>();
                    if (tex.count("path") > 0) texturePath = tex["path"].get<std::string>();
                }
                if (visual.count("visualization") > 0) {
                    const auto &viz = visual["visualization"];
                    showWireframe = viz.value("wireframe", showWireframe);
                    showNormals = viz.value("normals", showNormals);
                    showBoundingBox = viz.value("bounding_box", showBoundingBox);
                    normalLength = viz.value("normal_length", normalLength);
                }
                if (visual.count("debug") > 0) {
                    const auto &dbg = visual["debug"];
                    showContactPoints = dbg.value("contact_points", showContactPoints);
                    showInertiaEllipsoid = dbg.value("inertia_ellipsoid", showInertiaEllipsoid);
                    if (dbg.count("color") > 0) {
                        const auto &c = dbg["color"];
                        debugColorR = c.value("r", debugColorR);
                        debugColorG = c.value("g", debugColorG);
                        debugColorB = c.value("b", debugColorB);
                    }
                }

                g_visualProperties.addBodyProperties(
                    bodyIndex, r, g, b, transparency, smoothShade, edgeWidth,
                    showTextureSpace, textureParamName, texturePath,
                    shininess, reflectivity,
                    showWireframe, showNormals, normalLength, showBoundingBox,
                    showContactPoints, showInertiaEllipsoid,
                    debugColorR, debugColorG, debugColorB);
            }

            if (bodyJson.count("id") > 0) {
                int id = bodyJson["id"].get<int>();
                bodyMap[id] = body;
            }
            system.addBody(body);
            bodyIndex++;
        }

        // Process joints
        if (scenario.count("joints") > 0) {
            for (const auto &jointJson : scenario["joints"]) {
                int b0 = jointJson.at("body0_id").get<int>();
                int b1 = jointJson.at("body1_id").get<int>();
                if (bodyMap.find(b0) == bodyMap.end() || bodyMap.find(b1) == bodyMap.end()) {
                    std::cerr << "Invalid body IDs in joint definition" << std::endl;
                    continue;
                }
                Joint *joint = nullptr;
                std::string type = jointJson.at("type").get<std::string>();
                Eigen::Vector3f r0, r1;
                if (type == "hinge") {
                    r0 = {jointJson.at("body0_offset")["x"].get<float>(),
                          jointJson.at("body0_offset")["y"].get<float>(),
                          jointJson.at("body0_offset")["z"].get<float>()};
                    r1 = {jointJson.at("body1_offset")["x"].get<float>(),
                          jointJson.at("body1_offset")["y"].get<float>(),
                          jointJson.at("body1_offset")["z"].get<float>()};
                    Eigen::Quaternionf q0(
                        jointJson.at("body0_orientation")["w"].get<float>(),
                        jointJson.at("body0_orientation")["x"].get<float>(),
                        jointJson.at("body0_orientation")["y"].get<float>(),
                        jointJson.at("body0_orientation")["z"].get<float>());
                    Eigen::Quaternionf q1(
                        jointJson.at("body1_orientation")["w"].get<float>(),
                        jointJson.at("body1_orientation")["x"].get<float>(),
                        jointJson.at("body1_orientation")["y"].get<float>(),
                        jointJson.at("body1_orientation")["z"].get<float>());
                    joint = new Hinge(bodyMap[b0], bodyMap[b1], r0, q0, r1, q1);
                } else if (type == "spherical") {
                    r0 = {jointJson.at("body0_offset")["x"].get<float>(),
                          jointJson.at("body0_offset")["y"].get<float>(),
                          jointJson.at("body0_offset")["z"].get<float>()};
                    r1 = {jointJson.at("body1_offset")["x"].get<float>(),
                          jointJson.at("body1_offset")["y"].get<float>(),
                          jointJson.at("body1_offset")["z"].get<float>()};
                    joint = new Spherical(bodyMap[b0], bodyMap[b1], r0, r1);
                }
                if (joint) system.addJoint(joint);
            }
        }

        // Physics settings
        if (scenario.count("physics") > 0) {
            const auto &physics = scenario["physics"];

            // Set solver iterations
            if (physics.count("solver_iterations") > 0) {
                system.setSolverIterations(physics.at("solver_iterations").get<int>());
            }

            // Set gravity
            if (physics.count("gravity") > 0) {
                Eigen::Vector3f grav(
                    physics.at("gravity")["x"].get<float>(),
                    physics.at("gravity")["y"].get<float>(),
                    physics.at("gravity")["z"].get<float>());
                system.setGravity(grav);
                std::cout << "Set gravity to: [" << grav.x() << ", " << grav.y() << ", " << grav.z() << "]" << std::endl;
            }

            // Enhanced integration settings
            if (physics.count("integration") > 0) {
                const auto &integration = physics["integration"];

                // Set integration method
                if (integration.count("method") > 0) {
                    std::string methodStr = integration.at("method").get<std::string>();
                    IntegrationMethod method = IntegrationMethod::EXPLICIT_EULER; // Default

                    if (methodStr == "explicit_euler") {
                        method = IntegrationMethod::EXPLICIT_EULER;
                    } else if (methodStr == "symplectic_euler") {
                        method = IntegrationMethod::SYMPLECTIC_EULER;
                    } else if (methodStr == "verlet") {
                        method = IntegrationMethod::VERLET;
                    } else if (methodStr == "rk4") {
                        method = IntegrationMethod::RK4;
                    } else if (methodStr == "implicit_euler") {
                        method = IntegrationMethod::IMPLICIT_EULER;
                    } else {
                        std::cerr << "Unknown integration method: " << methodStr << ", defaulting to EXPLICIT_EULER" << std::endl;
                    }

                    system.setIntegrationMethod(method);
                    std::cout << "Set integration method to: " << methodStr << std::endl;
                }

                // Set solver type
                if (integration.count("solver_type") > 0) {
                    std::string solverStr = integration.at("solver_type").get<std::string>();
                    SolverType solver = SolverType::PGS; // Default

                    if (solverStr == "pgs") {
                        solver = SolverType::PGS;
                    } else if (solverStr == "pgssm") {
                        solver = SolverType::PGSSM;
                    } else if (solverStr == "conj_gradient") {
                        solver = SolverType::CONJ_GRADIENT;
                    } else if (solverStr == "conj_residual") {
                        solver = SolverType::CONJ_RESIDUAL;
                    } else {
                        std::cerr << "Unknown solver type: " << solverStr << ", defaulting to PGS" << std::endl;
                    }

                    system.setSolverType(solver);
                    std::cout << "Set solver type to: " << solverStr << std::endl;
                }

                // Graph coloring option
                if (integration.count("use_graph_coloring") > 0) {
                    bool useGraphColoring = integration.at("use_graph_coloring").get<bool>();
                    system.setUseGraphColoring(useGraphColoring);
                    std::cout << "Set graph coloring to: " << (useGraphColoring ? "enabled" : "disabled") << std::endl;
                }

                // Implicit Euler specific parameters
                if (integration.count("implicit_euler") > 0) {
                    const auto &implicitParams = integration["implicit_euler"];

                    if (implicitParams.count("damping") > 0) {
                        float damping = implicitParams.at("damping").get<float>();
                        system.setImplicitDamping(damping);
                        std::cout << "Set implicit Euler damping to: " << damping << std::endl;
                    }

                    if (implicitParams.count("gyroscopic_damping") > 0) {
                        float gyroDamping = implicitParams.at("gyroscopic_damping").get<float>();
                        system.setGyroscopicDamping(gyroDamping);
                        std::cout << "Set implicit Euler gyroscopic damping to: " << gyroDamping << std::endl;
                    }

                    if (implicitParams.count("max_velocity") > 0) {
                        float maxVel = implicitParams.at("max_velocity").get<float>();
                        system.setMaxLinearVelocity(maxVel);
                        std::cout << "Set implicit Euler max linear velocity to: " << maxVel << std::endl;
                    }

                    if (implicitParams.count("max_angular_velocity") > 0) {
                        float maxAngVel = implicitParams.at("max_angular_velocity").get<float>();
                        system.setMaxAngularVelocity(maxAngVel);
                        std::cout << "Set implicit Euler max angular velocity to: " << maxAngVel << std::endl;
                    }
                }
            }
        }

        return true;
    } catch (const std::exception &e) {
        std::cerr << "Error parsing scenario JSON: " << e.what() << std::endl;
        return false;
    }
}