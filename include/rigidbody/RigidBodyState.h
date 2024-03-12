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

    RigidBodyState(const RigidBody& _body);

    void restore(RigidBody& _body);

    void save(const RigidBody& _body);

private:

    Eigen::Vector3f x;          // position
    Eigen::Vector3f xdot;       // velocity
    Eigen::Quaternionf q;       // orientation
    Eigen::Vector3f omega;      // angular velocities

};

// Stores the state of an entire rigid body system
// 
// Note: dynamic elements, such as forces and torque, are not stored.
// The assumption is that forces and constraint impulses will be recomputed
// with the time step.
// 
// Collision detection will need to be performed after restore() is called.
//
class RigidBodySystemState
{
public:
    RigidBodySystemState(const RigidBodySystem& _system);

    void restore(RigidBodySystem& _system);

    void save(const RigidBodySystem& _body);

private: 
    std::vector<RigidBodyState> rigidBodyStates;

};
