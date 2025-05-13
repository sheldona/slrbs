#ifdef USE_PYBIND

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

// Core headers
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBodyState.h"
#include "collision/Geometry.h"
#include "collision/CollisionDetect.h"
#include "collision/AABB.h"
#include "collision/BVH.h"

// Contact and joint headers
#include "contact/Contact.h"
#include "contact/FaceContactTracker.h"
#include "joint/Joint.h"
#include "joint/Hinge.h"
#include "joint/Spherical.h"

// Utility headers
#include "util/MeshAssets.h"
#include "util/ScenarioLoader.h"
#include "util/OBJLoader.h"
#include "util/VisualProperties.h"
#include "util/Types.h"

// Solver headers
#include "solvers/Solver.h"
#include "solvers/SolverBoxBPP.h"
#include "solvers/SolverBoxPGS.h"
#include "solvers/SolverConjGradient.h"
#include "solvers/SolverConjResidual.h"
#include "solvers/SolverPGSSM.h"
#include "solvers/SolverProximal.h"

// Logging headers
#include "logging/SimDataLogger.h"

// Conditionally include AI model if available
#ifdef USE_TORCH
#include "ai/AIModel.h"
#endif

// Include polyscope for visualization if available
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"

namespace py = pybind11;

PYBIND11_MODULE(pyslrbs, m) {
    m.doc() = "Python bindings for SLRBS (Simple Linear Rigid Body Simulator)";

    // Add version information
    m.attr("__version__") = "0.1.0";

    // ----------------------------------------
    // Enum types
    // ----------------------------------------
    py::enum_<GeometryType>(m, "GeometryType")
        .value("Box", kBox)
        .value("Sphere", kSphere)
        .value("Cylinder", kCylinder)
        .value("Plane", kPlane)
        .export_values();

    py::enum_<IntegrationMethod>(m, "IntegrationMethod")
        .value("EXPLICIT_EULER", IntegrationMethod::EXPLICIT_EULER)
        .value("SYMPLECTIC_EULER", IntegrationMethod::SYMPLECTIC_EULER)
        .value("VERLET", IntegrationMethod::VERLET)
        .value("RK4", IntegrationMethod::RK4)
        .value("IMPLICIT_EULER", IntegrationMethod::IMPLICIT_EULER)
        .value("NEWTON", IntegrationMethod::NEWTON)
        .export_values();

    py::enum_<SolverType>(m, "SolverType")
        .value("PGS", SolverType::PGS)
        .value("PGSSM", SolverType::PGSSM)
        .value("CONJ_GRADIENT", SolverType::CONJ_GRADIENT)
        .value("CONJ_RESIDUAL", SolverType::CONJ_RESIDUAL)
        .value("BPP", SolverType::BPP)
        .export_values();

    // ----------------------------------------
    // Core mesh and geometry types
    // ----------------------------------------
    py::class_<Mesh>(m, "Mesh")
        .def(py::init<>())
        .def_readwrite("meshV", &Mesh::meshV)
        .def_readwrite("meshF", &Mesh::meshF);

    py::class_<Geometry>(m, "Geometry")
        .def("getType", &Geometry::getType)
        .def("computeInertia", &Geometry::computeInertia, py::arg("mass"));

    py::class_<Box, Geometry>(m, "Box")
        .def(py::init<const Eigen::Vector3f&>(), py::arg("dimensions"))
        .def("getDimensions", &Box::getDimensions);

    py::class_<Sphere, Geometry>(m, "Sphere")
        .def(py::init<float>(), py::arg("radius"))
        .def("getRadius", &Sphere::getRadius);

    py::class_<Cylinder, Geometry>(m, "Cylinder")
        .def(py::init<float, float>(), py::arg("height"), py::arg("radius"))
        .def("getHeight", &Cylinder::getHeight)
        .def("getRadius", &Cylinder::getRadius);

    py::class_<Plane, Geometry>(m, "Plane")
        .def(py::init<const Eigen::Vector3f&, const Eigen::Vector3f&>(),
             py::arg("point"), py::arg("normal"))
        .def("getPoint", &Plane::getPoint)
        .def("getNormal", &Plane::getNormal);

    // ----------------------------------------
    // Rigid body types and states
    // ----------------------------------------
    py::class_<RigidBodyState>(m, "RigidBodyState")
        .def(py::init<>())
        .def(py::init<const RigidBody&>())
        .def_readwrite("x", &RigidBodyState::x)
        .def_readwrite("xdot", &RigidBodyState::xdot)
        .def_readwrite("q", &RigidBodyState::q)
        .def_readwrite("omega", &RigidBodyState::omega)
        .def_readwrite("fixed", &RigidBodyState::fixed)
        .def_readwrite("color", &RigidBodyState::color)
        .def_readwrite("numColors", &RigidBodyState::numColors)
        .def_readwrite("f", &RigidBodyState::f)
        .def_readwrite("tau", &RigidBodyState::tau)
        .def("save", &RigidBodyState::save)
        .def("restore", &RigidBodyState::restore);

    py::class_<RigidBodySystemState>(m, "RigidBodySystemState")
        .def(py::init<>())
        .def(py::init<const RigidBodySystem&>())
        .def("save", &RigidBodySystemState::save)
        .def("restore", &RigidBodySystemState::restore)
        .def("canRestore", &RigidBodySystemState::canRestore);

    py::class_<RigidBody>(m, "RigidBody")
        .def(py::init<float, Geometry*, const std::string&>(),
             py::arg("mass"), py::arg("geometry"), py::arg("meshFile") = "")
        .def(py::init<float, Geometry*, const Mesh&>(),
             py::arg("mass"), py::arg("geometry"), py::arg("mesh"))
        .def_readwrite("x", &RigidBody::x)
        .def_readwrite("q", &RigidBody::q)
        .def_readwrite("xdot", &RigidBody::xdot)
        .def_readwrite("omega", &RigidBody::omega)
        .def_readwrite("mass", &RigidBody::mass)
        .def_readwrite("fixed", &RigidBody::fixed)
        .def_readwrite("restitution", &RigidBody::restitution)
        .def_readwrite("friction", &RigidBody::friction)
        .def_readwrite("density", &RigidBody::density)
        .def_readwrite("id", &RigidBody::id)
        .def_readwrite("color", &RigidBody::color)
        .def_readwrite("numColors", &RigidBody::numColors)
        .def_readwrite("visualProperties", &RigidBody::visualProperties)
        .def_property_readonly("I", [](const RigidBody& b) { return b.I; })
        .def_property_readonly("Iinv", [](const RigidBody& b) { return b.Iinv; })
        .def_property_readonly("Ibody", [](const RigidBody& b) { return b.Ibody; })
        .def_property_readonly("IbodyInv", [](const RigidBody& b) { return b.IbodyInv; })
        .def("applyForce", &RigidBody::addForceAtPos, py::arg("pos"), py::arg("force"))
        .def("applyImpulse", [](RigidBody& b, const Eigen::Vector3f& impulse) {
            b.xdot += impulse / b.mass;
        }, py::arg("impulse"))
        .def("applyTorque", [](RigidBody& b, const Eigen::Vector3f& torque) {
            b.tau += torque;
        }, py::arg("torque"))
        .def("computeInertiaTensor", &RigidBody::updateInertiaMatrix)
        .def("getVelocityAtPos", &RigidBody::getVelocityAtPos)
        .def("clearGeometricStiffness", &RigidBody::clearGeometricStiffness)
        .def("applyVisualProperties", &RigidBody::applyVisualProperties)
        .def("getContacts", [](RigidBody& self) -> const std::vector<Contact*>& {
            return self.contacts;
        }, py::return_value_policy::reference)
        .def("getJoints", [](RigidBody& self) -> const std::vector<Joint*>& {
            return self.joints;
        }, py::return_value_policy::reference)
        .def("getMesh", [](RigidBody& self) -> polyscope::SurfaceMesh* {
            return self.mesh;
        }, py::return_value_policy::reference);

    // ----------------------------------------
    // Joint classes
    // ----------------------------------------
    py::class_<Joint>(m, "Joint")
        .def("getBody0", &Joint::getBody0, py::return_value_policy::reference)
        .def("getBody1", &Joint::getBody1, py::return_value_policy::reference)
        .def_readwrite("dim", &Joint::dim)
        .def_readwrite("r0", &Joint::r0)
        .def_readwrite("r1", &Joint::r1)
        .def_readwrite("q0", &Joint::q0)
        .def_readwrite("q1", &Joint::q1)
        .def_property_readonly("lambda", [](Joint& j) -> const Eigen::VectorXf& { return j.lambda; })
        .def_property_readonly("phi", [](Joint& j) -> const Eigen::VectorXf& { return j.phi; })
        .def("computeJacobian", &Joint::computeJacobian)
        .def("computeGeometricStiffness", &Joint::computeGeometricStiffness);

    py::class_<Hinge, Joint>(m, "Hinge")
        .def(py::init<>())
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&,
             const Eigen::Quaternionf&, const Eigen::Vector3f&, const Eigen::Quaternionf&>(),
             py::arg("body0"), py::arg("body1"), py::arg("r0"), py::arg("q0"),
             py::arg("r1"), py::arg("q1"));

    py::class_<Spherical, Joint>(m, "Spherical")
        .def(py::init<>())
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Vector3f&>(),
             py::arg("body0"), py::arg("body1"), py::arg("r0"), py::arg("r1"));

    // ----------------------------------------
    // Contact classes
    // ----------------------------------------
    py::class_<Contact, Joint>(m, "Contact")
        .def(py::init<>())
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&,
             const Eigen::Vector3f&, float>(),
             py::arg("body0"), py::arg("body1"), py::arg("contactPoint"),
             py::arg("normal"), py::arg("penetration"))
        .def_readwrite("p", &Contact::p)
        .def_readwrite("n", &Contact::n)
        .def_readwrite("t", &Contact::t)
        .def_readwrite("b", &Contact::b)
        .def_readwrite("pene", &Contact::pene)
        .def_readwrite("relVel", &Contact::relVel)
        .def_readwrite("faceIndex0", &Contact::faceIndex0)
        .def_readwrite("faceIndex1", &Contact::faceIndex1)
        .def_readwrite("restitution", &Contact::restitution)
        .def_readwrite("bias", &Contact::bias)
        .def_readwrite("persistent", &Contact::persistent)
        .def_readwrite("k", &Contact::k)
        .def_static("setMu", [](float mu) { Contact::mu = mu; })
        .def_static("getMu", []() { return Contact::mu; })
        .def_static("setRestitutionThreshold", [](float rt) { Contact::restitutionThreshold = rt; })
        .def_static("getRestitutionThreshold", []() { return Contact::restitutionThreshold; })
        .def_static("setBaumgarte", [](float b) { Contact::baumgarte = b; })
        .def_static("getBaumgarte", []() { return Contact::baumgarte; })
        .def_static("setSlop", [](float s) { Contact::slop = s; })
        .def_static("getSlop", []() { return Contact::slop; })
        .def("setFaceIndices", &Contact::setFaceIndices)
        .def("computeContactFrame", &Contact::computeContactFrame)
        .def("computeJacobian", &Contact::computeJacobian)
        .def("warmStart", &Contact::warmStart)
        .def("reset", &Contact::reset);

    // FaceContactTracker
    py::class_<FaceContactTracker>(m, "FaceContactTracker")
        .def_static("initialize", &FaceContactTracker::initialize)
        .def_static("shutdown", &FaceContactTracker::shutdown)
        .def_static("initializeForBody", &FaceContactTracker::initializeForBody)
        .def_static("recordHit", &FaceContactTracker::recordHit)
        .def_static("reset", &FaceContactTracker::reset)
        .def_static("updateVisualization", &FaceContactTracker::updateVisualization)
        .def_static("setVisualizationEnabled", [](bool enabled) {
            FaceContactTracker::s_visualizationEnabled = enabled;
        })
        .def_static("isVisualizationEnabled", []() {
            return FaceContactTracker::s_visualizationEnabled;
        });

    // FaceHitData for tracking contact hits
    py::class_<FaceContactTracker::FaceHitData>(m, "FaceHitData")
        .def(py::init<>())
        .def_readwrite("hitCounts", &FaceContactTracker::FaceHitData::hitCounts)
        .def_readwrite("maxHits", &FaceContactTracker::FaceHitData::maxHits)
        .def("reset", &FaceContactTracker::FaceHitData::reset);

    // ----------------------------------------
    // BVH and Collision Detection
    // ----------------------------------------
    py::class_<AABB>(m, "AABB")
        .def(py::init<>())
        .def_readwrite("min", &AABB::min)
        .def_readwrite("max", &AABB::max)
        .def("contains", &AABB::contains)
        .def("intersects", &AABB::intersects)
        .def("merge", &AABB::merge)
        .def("getCenter", &AABB::getCenter)
        .def("getExtent", &AABB::getExtent);

    py::class_<BVHNode>(m, "BVHNode")
        .def(py::init<>())
        .def_readwrite("bounds", &BVHNode::bounds)
        .def_readwrite("primitiveIndices", &BVHNode::primitiveIndices);

    py::class_<BVH>(m, "BVH")
        .def(py::init<>())
        .def("build", &BVH::build)
        .def("visualize", &BVH::visualize)
        .def("setMaxDepth", [](BVH& bvh, int depth) { bvh.m_maxDepth = depth; })
        .def("getMaxDepth", [](const BVH& bvh) { return bvh.m_maxDepth; });

    py::class_<CollisionDetect>(m, "CollisionDetect")
        .def(py::init<RigidBodySystem*>())
        .def("clear", &CollisionDetect::clear)
        .def("detectCollisions", &CollisionDetect::detectCollisions)
        .def("computeContactJacobians", &CollisionDetect::computeContactJacobians)
        .def("getContacts", [](CollisionDetect& self) -> const std::vector<Contact*>& {
            return self.getContacts();
        }, py::return_value_policy::reference);

    // ----------------------------------------
    // Solver Classes
    // ----------------------------------------
    py::class_<Solver>(m, "Solver")
        .def("solve", &Solver::solve)
        .def("setMaxIter", &Solver::setMaxIter)
        .def("getMaxIter", &Solver::getMaxIter);

    py::class_<SolverBoxPGS, Solver>(m, "SolverBoxPGS")
        .def(py::init<RigidBodySystem*>());

    py::class_<SolverBoxBPP, Solver>(m, "SolverBoxBPP")
        .def(py::init<RigidBodySystem*>());

    py::class_<SolverConjGradient, Solver>(m, "SolverConjGradient")
        .def(py::init<RigidBodySystem*>());

    py::class_<SolverConjResidual, Solver>(m, "SolverConjResidual")
        .def(py::init<RigidBodySystem*>());

    py::class_<SolverPGSSM, Solver>(m, "SolverPGSSM")
        .def(py::init<RigidBodySystem*>());

    py::class_<SolverProximal, Solver>(m, "SolverProximal")
        .def(py::init<RigidBodySystem*>());

    // ----------------------------------------
    // RigidBodySystem - Main Simulation Class
    // ----------------------------------------
    py::class_<RigidBodySystem>(m, "RigidBodySystem")
        .def(py::init<>())
        .def("addBody", &RigidBodySystem::addBody, py::keep_alive<1, 2>())
        .def("addJoint", &RigidBodySystem::addJoint, py::keep_alive<1, 2>())
        .def("setGravity", &RigidBodySystem::setGravity)
        .def("getGravity", &RigidBodySystem::getGravity)
        .def("step", &RigidBodySystem::step, py::arg("dt") = 1.0f/60.0f)
        .def("clear", &RigidBodySystem::clear)
        .def("setIntegrationMethod", &RigidBodySystem::setIntegrationMethod)
        .def("getIntegrationMethod", &RigidBodySystem::getIntegrationMethod)
        .def("setSolverType", &RigidBodySystem::setSolverType)
        .def("getSolverType", &RigidBodySystem::getSolverType)
        .def("setSolverIterations", &RigidBodySystem::setSolverIterations)
        .def("getSolverIterations", &RigidBodySystem::getSolverIterations)
        .def("setUseGraphColoring", &RigidBodySystem::setUseGraphColoring)
        .def("getUseGraphColoring", &RigidBodySystem::getUseGraphColoring)
        .def("setImplicitDamping", &RigidBodySystem::setImplicitDamping)
        .def("getImplicitDamping", &RigidBodySystem::getImplicitDamping)
        .def("setGyroscopicDamping", &RigidBodySystem::setGyroscopicDamping)
        .def("getGyroscopicDamping", &RigidBodySystem::getGyroscopicDamping)
        .def("setMaxLinearVelocity", &RigidBodySystem::setMaxLinearVelocity)
        .def("getMaxLinearVelocity", &RigidBodySystem::getMaxLinearVelocity)
        .def("setMaxAngularVelocity", &RigidBodySystem::setMaxAngularVelocity)
        .def("getMaxAngularVelocity", &RigidBodySystem::getMaxAngularVelocity)
        .def("enableLimitVelocities", &RigidBodySystem::enableLimitVelocities)
        .def("isLimitVelocitiesEnabled", &RigidBodySystem::isLimitVelocitiesEnabled)
        .def("enableCollisions", &RigidBodySystem::enableCollisions)
        .def("isCollisionsEnabled", &RigidBodySystem::isCollisionsEnabled)
        .def("enableGSDamping", &RigidBodySystem::enableGSDamping)
        .def("isGSDampingEnabled", &RigidBodySystem::isGSDampingEnabled)
        .def("setGSAlpha", &RigidBodySystem::setGSAlpha)
        .def("getGSAlpha", &RigidBodySystem::getGSAlpha)
        .def("getBodies", [](RigidBodySystem& self) {
            return self.getBodies();
        }, py::return_value_policy::reference)
        .def("getJoints", [](RigidBodySystem& self) {
            return self.getJoints();
        }, py::return_value_policy::reference)
        .def("getContacts", [](RigidBodySystem& self) {
            return self.getContacts();
        }, py::return_value_policy::reference)
        .def("setPreStepFunction", [](RigidBodySystem& self,
                                       const std::function<void(RigidBodySystem&, float)>& func) {
            self.setPreStepFunction(func);
        })
        .def("setResetFunction", [](RigidBodySystem& self,
                                     const std::function<void()>& func) {
            self.setResetFunction(func);
        })
        .def("computeInertias", &RigidBodySystem::computeInertias);

    // ----------------------------------------
    // Utility functions for mesh creation
    // ----------------------------------------
    m.def("createBox", &createBox, py::arg("dimensions"),
          "Create a box mesh with the given dimensions");
    m.def("createSphere", &createSphere, py::arg("radius"),
          "Create a sphere mesh with the given radius");
    m.def("createCylinder", &createCylinder, py::arg("N"), py::arg("radius"), py::arg("height"),
          "Create a cylinder mesh with N sides, given radius and height");
    m.def("createPlane", &createPlane, py::arg("point"), py::arg("normal"),
          "Create a plane mesh with the given point and normal");

    // ----------------------------------------
    // Scenario Loader
    // ----------------------------------------
    py::class_<ScenarioLoader>(m, "ScenarioLoader")
        .def_static("getScenariosPath", &ScenarioLoader::getScenariosPath)
        .def_static("listAvailableScenarios", &ScenarioLoader::listAvailableScenarios)
        .def_static("loadFromFile", &ScenarioLoader::loadFromFile,
                   py::arg("system"), py::arg("filename"),
                   "Load a scenario from a JSON file into the given RigidBodySystem")
        .def_static("parseScenario", &ScenarioLoader::parseScenario,
                   py::arg("system"), py::arg("jsonContent"),
                   "Parse a scenario from JSON content into the given RigidBodySystem");

    // ----------------------------------------
    // OBJ Loader for mesh files
    // ----------------------------------------
    py::class_<OBJLoader>(m, "OBJLoader")
        .def_static("load", &OBJLoader::load,
                   py::arg("filename"), py::arg("meshV"), py::arg("meshF"),
                   "Load an OBJ file into the given vertex and face matrices");

    // ----------------------------------------
    // MeshAssetRegistry for caching loaded meshes
    // ----------------------------------------
    py::class_<MeshAssetRegistry>(m, "MeshAssetRegistry")
        .def_static("loadObj", &MeshAssetRegistry::loadObj, py::arg("filename"),
                   py::return_value_policy::reference)
        .def_static("clear", &MeshAssetRegistry::clear)
        .def_static("getCachedMeshes", []() -> MeshCache& {
            return MeshAssetRegistry::cachedMeshes();
        }, py::return_value_policy::reference);

    // ----------------------------------------
    // Visual Properties
    // ----------------------------------------
    py::class_<BodyVisualProperties>(m, "BodyVisualProperties")
        .def(py::init<>())
        .def_readwrite("bodyIndex", &BodyVisualProperties::bodyIndex)
        .def_readwrite("colorR", &BodyVisualProperties::colorR)
        .def_readwrite("colorG", &BodyVisualProperties::colorG)
        .def_readwrite("colorB", &BodyVisualProperties::colorB)
        .def_readwrite("transparency", &BodyVisualProperties::transparency)
        .def_readwrite("smoothShade", &BodyVisualProperties::smoothShade)
        .def_readwrite("edgeWidth", &BodyVisualProperties::edgeWidth)
        .def_readwrite("showTextureSpace", &BodyVisualProperties::showTextureSpace)
        .def_readwrite("textureParamName", &BodyVisualProperties::textureParamName)
        .def_readwrite("texturePath", &BodyVisualProperties::texturePath)
        .def_readwrite("shininess", &BodyVisualProperties::shininess)
        .def_readwrite("reflectivity", &BodyVisualProperties::reflectivity)
        .def_readwrite("showWireframe", &BodyVisualProperties::showWireframe)
        .def_readwrite("showNormals", &BodyVisualProperties::showNormals)
        .def_readwrite("normalLength", &BodyVisualProperties::normalLength)
        .def_readwrite("showBoundingBox", &BodyVisualProperties::showBoundingBox);

    py::class_<ScenarioVisualProperties>(m, "ScenarioVisualProperties")
        .def(py::init<>())
        .def("clear", &ScenarioVisualProperties::clear)
        .def("addBodyProperties",
            py::overload_cast<int, float, float, float, float, bool, float>(
                &ScenarioVisualProperties::addBodyProperties),
            py::arg("bodyIndex"), py::arg("r"), py::arg("g"), py::arg("b"),
            py::arg("transparency"), py::arg("smoothShade"), py::arg("edgeWidth"))
        .def("addBodyProperties",
            py::overload_cast<int, float, float, float, float, bool, float, bool,
                               const std::string&, const std::optional<std::string>&,
                               float, float, bool, bool, float, bool, bool, bool,
                               float, float, float>(
                &ScenarioVisualProperties::addBodyProperties),
            py::arg("bodyIndex"), py::arg("r"), py::arg("g"), py::arg("b"),
            py::arg("transparency"), py::arg("smoothShade"), py::arg("edgeWidth"),
            py::arg("showTextureSpace") = false, py::arg("textureParamName") = "uv_coords",
            py::arg("texturePath") = std::nullopt, py::arg("shininess") = 0.0f,
            py::arg("reflectivity") = 0.0f, py::arg("showWireframe") = false,
            py::arg("showNormals") = false, py::arg("normalLength") = 0.1f,
            py::arg("showBoundingBox") = false, py::arg("showContactPoints") = false,
            py::arg("showInertiaEllipsoid") = false, py::arg("debugColorR") = 1.0f,
            py::arg("debugColorG") = 1.0f, py::arg("debugColorB") = 0.0f)
        .def("findPropertiesForBody", &ScenarioVisualProperties::findPropertiesForBody,
            py::arg("bodyIndex"), py::return_value_policy::reference);

    // Expose the global visual properties instance
    m.attr("g_visualProperties") = py::cast(&g_visualProperties, py::return_value_policy::reference);

    // ----------------------------------------
    // SimDataLogger for logging simulation data
    // ----------------------------------------
    py::class_<slrbs::SimDataLogger>(m, "SimDataLogger")
        .def(py::init<>())
        .def("initialize", &slrbs::SimDataLogger::initialize, py::arg("basePath"))
        .def("startLogging", &slrbs::SimDataLogger::startLogging,
             py::arg("name"), py::arg("append") = false)
        .def("beginFrame", &slrbs::SimDataLogger::beginFrame)
        .def("endFrame", &slrbs::SimDataLogger::endFrame)
        .def("logMatrix3f", &slrbs::SimDataLogger::logMatrix3f,
             py::arg("label"), py::arg("mat"))
        .def("logMatrix", &slrbs::SimDataLogger::logMatrix,
             py::arg("label"), py::arg("mat"))
        .def("logScalar", &slrbs::SimDataLogger::logScalar,
             py::arg("label"), py::arg("value"))
        .def("logInt", &slrbs::SimDataLogger::logInt,
             py::arg("label"), py::arg("value"));

    // ----------------------------------------
    // Conditional Features
    // ----------------------------------------
    // Add OpenMP status
    #ifdef _OPENMP
    m.attr("has_openmp") = true;
    #else
    m.attr("has_openmp") = false;
    #endif

    // Add LibTorch AI model if available
    #ifdef USE_TORCH
    m.attr("has_torch") = true;
    py::class_<AIModel>(m, "AIModel")
        .def(py::init<>())
        .def("loadModel", &AIModel::loadModel, py::arg("modelPath"))
        .def("predict", &AIModel::predict, py::arg("input"))
        .def("supportsBatch", &AIModel::supportsBatch)
        .def("getInputDim", &AIModel::getInputDim)
        .def("getOutputDim", &AIModel::getOutputDim);
    #else
    m.attr("has_torch") = false;
    #endif

    // Add OpenCV status
    #ifdef USE_OPENCV
    m.attr("has_opencv") = true;
    #else
    m.attr("has_opencv") = false;
    #endif

    // Add Qt support status
    #ifdef USE_QT
    m.attr("has_qt") = true;
    #else
    m.attr("has_qt") = false;
    #endif

    // Add SIMD support status
    #ifdef USE_SIMD_INTRINSICS
    m.attr("has_simd") = true;
    #else
    m.attr("has_simd") = false;
    #endif

    // ----------------------------------------
    // Polyscope Integration
    // ----------------------------------------
    // Initialize and shutdown Polyscope
    m.def("initPolyscope", []() {
        polyscope::init();
    }, "Initialize the Polyscope visualization library");

    m.def("showPolyscope", []() {
        polyscope::show();
    }, "Show the Polyscope UI and enter the main loop");

    m.def("shutdownPolyscope", []() {
        polyscope::shutdown();
    }, "Shutdown the Polyscope visualization library");

    // Visualize the whole physics system
    m.def("visualizeSystem", [](RigidBodySystem& system, const std::string& name = "Physics System") {
        // Register meshes for all bodies that have them
        for (auto* body : system.getBodies()) {
            if (body->mesh) {
                body->applyVisualProperties();
            }
        }

        // Update visualization for face contact tracking
        FaceContactTracker::updateVisualization(&system);

        return true;
    }, py::arg("system"), py::arg("name") = "Physics System",
    "Visualize the physics system using Polyscope");

    // ----------------------------------------
    // Helper functions
    // ----------------------------------------
    // Create and run a scenario
    m.def("createAndSimulateScene", [](const std::string& scenarioFile, float duration, float dt) {
        // Initialize a new system
        auto system = std::make_unique<RigidBodySystem>();

        // Load the scenario
        if (!ScenarioLoader::loadFromFile(*system, scenarioFile)) {
            throw std::runtime_error("Failed to load scenario: " + scenarioFile);
        }

        // Determine number of steps
        int steps = static_cast<int>(duration / dt);
        py::print("Simulating", steps, "steps...");

        // Allow for progress monitoring
        py::gil_scoped_release release;
        for (int i = 0; i < steps; i++) {
            system->step(dt);

            // Every 100 steps, print progress
            if (i % 100 == 0) {
                py::gil_scoped_acquire acquire;
                py::print("Step", i, "/", steps);
            }
        }

        return system;
    }, py::arg("scenarioFile"), py::arg("duration") = 10.0f, py::arg("dt") = 1.0f/60.0f,
    py::return_value_policy::take_ownership,
    "Load a scenario and run a simulation for the specified duration");

    // Create a simple scene with a box falling onto a plane
    m.def("createSimpleScene", []() {
        auto system = std::make_unique<RigidBodySystem>();
        system->setGravity(Eigen::Vector3f(0, -9.81f, 0));

        // Create a ground plane
        auto* planeGeom = new Plane(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0));
        auto* plane = new RigidBody(0.0f, planeGeom, createPlane(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0)));
        plane->fixed = true;
        system->addBody(plane);

        // Create a falling box
        auto* boxGeom = new Box(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
        auto* box = new RigidBody(1.0f, boxGeom, createBox(Eigen::Vector3f(1.0f, 1.0f, 1.0f)));
        box->x = Eigen::Vector3f(0, 5.0f, 0);
        box->restitution = 0.5f;
        box->friction = 0.5f;
        system->addBody(box);

        return system;
    }, py::return_value_policy::take_ownership,
    "Create a simple scene with a box falling onto a plane");

    // Export CSV data from a simulation
    m.def("exportSimulationData", [](RigidBodySystem& system, const std::string& filename, int numSteps, float dt) {
        slrbs::SimDataLogger logger;
        logger.initialize("./logs");
        logger.startLogging(filename);

        for (int i = 0; i < numSteps; i++) {
            logger.beginFrame();

            // Log positions and velocities
            for (size_t j = 0; j < system.getBodies().size(); j++) {
                auto* body = system.getBodies()[j];
                logger.logMatrix("position_" + std::to_string(j),
                                Eigen::Matrix<float, 1, 3>(body->x));
                logger.logMatrix("velocity_" + std::to_string(j),
                                Eigen::Matrix<float, 1, 3>(body->xdot));
                logger.logMatrix("angular_velocity_" + std::to_string(j),
                                Eigen::Matrix<float, 1, 3>(body->omega));
            }

            // Step simulation
            system.step(dt);
            logger.endFrame();
        }

        py::print("Exported simulation data to:", "./logs/" + filename + ".csv");
        return true;
    }, py::arg("system"), py::arg("filename"), py::arg("numSteps"), py::arg("dt") = 1.0f/60.0f,
    "Export simulation data to a CSV file");
}
#endif