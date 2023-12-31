#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"

float Contact::mu = 0.8f;

Contact::Contact() : Joint(), p(), n(), t(), b()
{

}

Contact::Contact(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _p, const Eigen::Vector3f& _n, float _pene) :
    Joint(_body0, _body1),
    p(_p), n(_n), t(), b(), pene(_pene)
{
    dim = 3;
    J0.setZero(3, 6);
    J1.setZero(3, 6);
    J0Minv.setZero(3, 6);
    J1Minv.setZero(3, 6);
    lambda.setZero(3);
    phi.setZero(3);
    phi(0) = _pene;

    body0->contacts.push_back(this);
    body1->contacts.push_back(this);
}

Contact::~Contact()
{

}

void Contact::computeContactFrame()
{
    // Compute the contact frame, which consists of an orthonormal
    //  bases formed the vector n, t, and b
    //

    // TODO Compute first tangent direction t
    //
    t = n.cross(Eigen::Vector3f(1, 0, 0));
    if ( t.norm() < 1e-5f )
    {
        // Fail-safe: use axis-aligned direction (0,0,-1) 
        t = n.cross(Eigen::Vector3f(0, 1, 0));
    }
    t.normalize();

    // TODO Compute second tangent direction b.
    //
    b = n.cross(t);
    b.normalize();
}

void Contact::computeJacobian()
{
    // Compute the Jacobians J0 and J1
    //
    const Eigen::Vector3f rr0 = p - body0->x;
    const Eigen::Vector3f rr1 = p - body1->x;

    J0.setZero(3, 6);
    J1.setZero(3, 6);
    J0Minv.setZero(3, 6);
    J1Minv.setZero(3, 6);
    lambda.setZero(3);
    phi.setZero(3);
    phi(0) = pene;

    // TODO: assemble the contact Jacobian
    // 
    // normal row (non-interpenetration)
    J0.block(0, 0, 1, 3) = n.transpose();
    J0.block(0, 3, 1, 3) = rr0.cross(n).transpose();
    J1.block(0, 0, 1, 3) = -n.transpose();
    J1.block(0, 3, 1, 3) = -rr1.cross(n).transpose();
    // tangent 1 (friction)
    J0.block(1, 0, 1, 3) = t.transpose();
    J0.block(1, 3, 1, 3) = rr0.cross(t).transpose();
    J1.block(1, 0, 1, 3) = -t.transpose();
    J1.block(1, 3, 1, 3) = -rr1.cross(t).transpose();
    // tangent 2 (friction)
    J0.block(2, 0, 1, 3) = b.transpose();
    J0.block(2, 3, 1, 3) = rr0.cross(b).transpose();
    J1.block(2, 0, 1, 3) = -b.transpose();
    J1.block(2, 3, 1, 3) = -rr1.cross(b).transpose();

    // Finally, compute the blocks J M^-1 for each body.
    //
    J0Minv.block(0,0,3,3) = (1.0f/body0->mass) * J0.block(0, 0, 3, 3);
    J0Minv.block(0,3,3,3) = J0.block(0, 3, 3, 3) * body0->Iinv;
    J1Minv.block(0,0,3,3) = (1.0f/body1->mass) * J1.block(0, 0, 3, 3);
    J1Minv.block(0,3,3,3) = J1.block(0, 3, 3, 3) * body1->Iinv;
}
