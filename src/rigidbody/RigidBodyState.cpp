#include "rigidbody/RigidBodyState.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <cassert>
#include <stdexcept>

RigidBodyState::RigidBodyState()
    : x(Eigen::Vector3f::Zero())
    , xdot(Eigen::Vector3f::Zero())
    , q(Eigen::Quaternionf::Identity())
    , omega(Eigen::Vector3f::Zero())
    , fixed(false)
    , color(-1)
    , numColors(0)
    , f(Eigen::Vector3f::Zero())
    , tau(Eigen::Vector3f::Zero())
{
}

RigidBodyState::RigidBodyState(const RigidBody& body)
{
    save(body);
}

void RigidBodyState::save(const RigidBody& body)
{
    // Save kinematic state
    x = body.x;
    xdot = body.xdot;
    q = body.q;
    omega = body.omega;

    // Save properties
    fixed = body.fixed;
    color = body.color;
    numColors = body.numColors;

    // Save force state
    f = body.f;
    tau = body.tau;
}

void RigidBodyState::restore(RigidBody& body)
{
    // Restore kinematic state
    body.x = x;
    body.xdot = xdot;

    // Ensure quaternion is normalized to prevent numerical drift
    body.q = q.normalized();
    body.omega = omega;

    // Restore properties
    body.fixed = fixed;
    body.color = color;
    body.numColors = numColors;

    // Restore forces from saved state
    body.f = f;
    body.tau = tau;

    // Clear constraint forces - these will be recomputed
    body.fc.setZero();
    body.tauc.setZero();

    // Remove contacts - these will be redetected
    body.contacts.clear();

    // Update inertia matrices since orientation may have changed
    body.updateInertiaMatrix();
}

RigidBodySystemState::RigidBodySystemState(const RigidBodySystem& system)
{
    save(system);
}

void RigidBodySystemState::save(const RigidBodySystem& system)
{
    const auto& bodies = system.getBodies();
    rigidBodyStates.resize(bodies.size());

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        rigidBodyStates[i].save(*(bodies[i]));
    }
}

bool RigidBodySystemState::canRestore(const RigidBodySystem& system) const
{
    return system.getBodies().size() == rigidBodyStates.size();
}

void RigidBodySystemState::restore(RigidBodySystem& system)
{
    const auto& bodies = system.getBodies();

    if (!canRestore(system))
    {
        throw std::runtime_error("Cannot restore state: number of bodies in system ("
            + std::to_string(bodies.size()) + ") differs from saved state ("
            + std::to_string(rigidBodyStates.size()) + ")");
    }

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        rigidBodyStates[i].restore(*(bodies[i]));
    }

    // Clear contacts - they will be recomputed during collision detection
    system.getContacts().clear();

    // Note: We don't clear joints as they are typically structural
    // and not dynamically created/destroyed like contacts
}