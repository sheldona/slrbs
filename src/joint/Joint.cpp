#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"

Joint::Joint() :
    body0(nullptr), body1(nullptr)
{

}


Joint::Joint(RigidBody* _body0, RigidBody* _body1) :
    body0(_body0), body1(_body1)
{

}

