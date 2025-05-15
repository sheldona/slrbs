#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "integrator/IntegrationMethod.h"

// Forward declarations
class RigidBodySystem;
class Contact;
class CollisionDetect;
class Joint;
class Solver;
class RigidBody;
class RigidBodySystemState;


// Pre-step callback signature (system reference and timestep)
typedef std::function<void(RigidBodySystem& system, float dt)> PreStepFunc;
// Reset callback signature
typedef std::function<void()> ResetFunc;

// UI‑compatible enum that maps to IntegrationMethod
enum class IntegratorType {
    EXPLICIT_EULER = 0,
    SYMPLECTIC_EULER = 1,
    VERLET = 2,
    RK4 = 3,
    IMPLICIT_EULER = 4,
    NEWTON = 5
};

// Solver types for constraint solving
enum class SolverType {
    BPP,
    PGS,
    CONJ_GRADIENT,
    CONJ_RESIDUAL,
    PGSSM,
    PROXIMAL
};

class RigidBodySystem
{
public:
    // Constructor/destructor
    RigidBodySystem();
    virtual ~RigidBodySystem();

    // Advance the simulation by dt
    void step(float dt);

    // Remove all bodies/joints and reset
    void clear();

    // Add a rigid body (system takes ownership)
    void addBody(RigidBody* b);

    // Add a joint (system takes ownership)
    void addJoint(Joint* j);

    // Accessors
    const std::vector<RigidBody*>& getBodies() const { return m_bodies; }
    std::vector<RigidBody*>&       getBodies()       { return m_bodies; }
    const std::vector<Contact*>&   getContacts() const;
    std::vector<Contact*>&         getContacts();
    const std::vector<Joint*>&     getJoints()  const { return m_joints; }
    std::vector<Joint*>&           getJoints()        { return m_joints; }

    // Callbacks
    void setPreStepFunc(PreStepFunc func) { m_preStepFunc = std::move(func); }
    void setResetFunc  (ResetFunc  func) { m_resetFunc  = std::move(func); }

    // Collision toggle
    void setEnableCollisionDetection(bool enable) { m_collisionsEnabled = enable; }
    bool getEnableCollisionDetection() const      { return m_collisionsEnabled; }

    // Graph coloring toggle
    void setUseGraphColoring(bool enable) { m_useGraphColoring = enable; }
    bool getUseGraphColoring() const      { return m_useGraphColoring; }

    // Gravity
    void setGravity(const Eigen::Vector3f& g) { m_gravity = g; }
    const Eigen::Vector3f& getGravity() const { return m_gravity; }

    // Solver settings
    void setSolverType     (SolverType type) { m_solverType = type; }
    SolverType getSolverType() const         { return m_solverType; }
    void setSolverIterations(int iters)      { m_solverIter = iters; }
    int  getSolverIterations() const         { return m_solverIter; }

    // Integration‑method interface
    void setIntegrationMethod(IntegrationMethod method) { m_integrationMethod = method; }
    IntegrationMethod getIntegrationMethod() const      { return m_integrationMethod; }
    void setIntegratorType(IntegratorType type)         { m_integrationMethod = static_cast<IntegrationMethod>(type); }
    IntegratorType getIntegratorType() const            { return static_cast<IntegratorType>(m_integrationMethod); }

    // Implicit‑Euler parameter controls
    void  setImplicitDamping   (float d) { m_implicitDamping    = d; }
    float getImplicitDamping() const     { return m_implicitDamping; }

    void  setGyroscopicDamping (float d) { m_gyroDamping        = d; }
    float getGyroscopicDamping() const   { return m_gyroDamping; }

    void  setMaxLinearVelocity (float v) { m_maxLinearVelocity  = v; }
    float getMaxLinearVelocity() const   { return m_maxLinearVelocity; }

    void  setMaxAngularVelocity(float v) { m_maxAngularVelocity = v; }
    float getMaxAngularVelocity() const  { return m_maxAngularVelocity; }

    // Optionally disable velocity limiting entirely
    void setVelocityLimitingEnabled(bool e) { m_limitVelocities = e; }
    bool getVelocityLimitingEnabled() const { return m_limitVelocities; }

    // Geometric stiffness damping controls
    void setGeometricStiffnessDampingEnabled(bool e) { m_enableGSDamping = e; }
    bool getGeometricStiffnessDampingEnabled() const { return m_enableGSDamping; }

    void setGeometricStiffnessAlpha(float a) { m_gsAlpha = a; }
    float getGeometricStiffnessAlpha() const { return m_gsAlpha; }

    void setUseOpenMP(bool enable) { m_useOpenMP = enable; }
    bool getUseOpenMP() const { return m_useOpenMP; }

    void setUseSolverOpenMP(bool enable) { m_useSolverOpenMP = enable; }
    bool getUseSolverOpenMP() const { return m_useSolverOpenMP; }
    void setUseCollisionOpenMP(bool enable) { m_useCollisionOpenMP = enable; }
    bool getUseCollisionOpenMP() const { return m_useCollisionOpenMP; }

    // void setNewtonMaxIterations(int iters);
    // void setNewtonTolerance(float tol);
    // void setNewtonDamping(float damping);

    // // For Proximal solver
    // void setProximalAbsTolerance(float tol);
    // void setProximalRelTolerance(float tol);
    // Solver* getProximalSolver();
    //
    // // For conjugate gradient solvers
    // void setConjTolerance(float tol);
    // float getConjTolerance() const;
    // void setConjRestartInterval(int interval);
    // int getConjRestartInterval() const;

    // void setMaxIterations(int iterations) { m_maxIterations = iterations; }
    // void setTolerance(float tolerance) { m_tolerance = tolerance; }
    // void setDamping(float damping) { m_damping = damping; }

private:
    // Internal pipeline methods
    void computeInertias();
    void calcConstraintForces(float dt);

    // Members
    std::vector<RigidBody*>      m_bodies;
    std::vector<Joint*>          m_joints;
    std::unique_ptr<CollisionDetect> m_collisionDetect;

    bool m_collisionsEnabled = true;
    bool m_useGraphColoring  = true;
    bool m_limitVelocities   = true;

    PreStepFunc m_preStepFunc = nullptr;
    ResetFunc   m_resetFunc   = nullptr;

    Eigen::Vector3f m_gravity = {0.0f, -9.81f, 0.0f};

    SolverType        m_solverType  = SolverType::PGS;
    int               m_solverIter  = 10;
    IntegrationMethod m_integrationMethod = IntegrationMethod::EXPLICIT_EULER;

    // Implicit‑Euler parameters
    float m_implicitDamping    = 0.98f;
    float m_gyroDamping        = 0.20f;
    float m_maxLinearVelocity  = 50.0f;
    float m_maxAngularVelocity = 20.0f;

    // Geometric stiffness damping
    bool  m_enableGSDamping = false;  // enable geometric‑stiffness based damping
    float m_gsAlpha         = 0.0f;   // geometric stiffness coefficient

    bool m_useOpenMP = true;
    bool m_useSolverOpenMP = true;
    bool m_useCollisionOpenMP = true;

    int m_maxIterations = 10;
    float m_tolerance = 1e-6f;
    float m_damping = 0.5f;
};