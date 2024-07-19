#include "rigidbody/RigidBodyState.h"

#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <cassert>

RigidBodyState::RigidBodyState()
{

}

RigidBodyState::RigidBodyState(const RigidBody& _body) : x(_body.x), xdot(_body.xdot), q(_body.q), omega(_body.omega)
{
    save(_body);
}

void RigidBodyState::save(const RigidBody& _body)
{
    x = _body.x; 
    xdot = _body.xdot; 
    q = _body.q; 
    omega = _body.omega;
}

void RigidBodyState::restore(RigidBody& _body)
{
    // Set kinematic state
    _body.x = x;
    _body.xdot = xdot;
    _body.q = q;
    _body.omega = omega;

    // Clear forces, these should be recomputed during RigidBodySystem::step()
    _body.f.setZero();
    _body.fc.setZero();

    // remove contacts
    _body.contacts.clear();
}


RigidBodySystemState::RigidBodySystemState(const RigidBodySystem& _system)
{
    save(_system);
}

void RigidBodySystemState::save(const RigidBodySystem& _system)
{
    const auto& bodies = _system.getBodies();
    const unsigned int nBodies = bodies.size();
    rigidBodyStates.resize(nBodies);
    for (unsigned int i = 0; i < nBodies; ++i)
    {
        rigidBodyStates[i] = RigidBodyState(*(bodies[i]));
    }
}

void RigidBodySystemState::restore(RigidBodySystem& _system)
{
    const auto& bodies = _system.getBodies();
    const unsigned int nBodies = bodies.size();

    assert((void("RigidBodySystemState::restore: number of rigid bodies is different than restore state."), nBodies == rigidBodyStates.size()));

    for (unsigned int i = 0; i < nBodies; ++i)
    {
        rigidBodyStates[i].restore(*(bodies[i]));
    }

    _system.getContacts().clear();
}

