#pragma once

#include <util/Types.h>
#include <vector>

class RigidBody;
class RigidBodySystem;

// Stores the kinematic state of a single rigid body.
//
class RigidBodyState
{
public:
    RigidBodyState();
    RigidBodyState(const RigidBody& body);

    void restore(RigidBody& body);
    void save(const RigidBody& body);

private:
    // Kinematic state
    Eigen::Vector3f x = Eigen::Vector3f::Zero();          // Position
    Eigen::Vector3f xdot = Eigen::Vector3f::Zero();       // Velocity
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity(); // Orientation
    Eigen::Vector3f omega = Eigen::Vector3f::Zero();      // Angular velocity

    // Properties
    bool fixed = false;                 // Static flag
    int color = -1;                     // Graph coloring data
    int numColors = 0;                  // Total colors in the system

    // Force state
    Eigen::Vector3f f = Eigen::Vector3f::Zero();          // Linear force
    Eigen::Vector3f tau = Eigen::Vector3f::Zero();        // Angular force (torque)
};

// Stores the state of an entire rigid body system
//
// Note: dynamic elements, such as forces and constraint impulses will be
// either restored or recomputed with the time step depending on the
// restoration policy.
//
// Collision detection will need to be performed after restore() is called.
//
class RigidBodySystemState
{
public:
    RigidBodySystemState() = default;
    explicit RigidBodySystemState(const RigidBodySystem& system);

    void restore(RigidBodySystem& system);
    void save(const RigidBodySystem& system);

    // Helper method to check if the state can be restored
    bool canRestore(const RigidBodySystem& system) const;

    // Get number of bodies in this state
    size_t getBodyCount() const { return rigidBodyStates.size(); }

private:
    std::vector<RigidBodyState> rigidBodyStates;
};