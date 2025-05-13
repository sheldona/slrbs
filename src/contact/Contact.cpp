#include "contact/Contact.h"
#include "contact/FaceContactTracker.h"
#include "rigidbody/RigidBody.h"

// Initialize static members
float Contact::mu                   = 0.8f;
float Contact::restitutionThreshold = 0.5f;
float Contact::baumgarte            = 0.2f;
float Contact::slop                  = 0.01f;

namespace {
    static inline Eigen::Matrix3f prodOfCrossProd(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
    {
        float a0b0 = a.x() * b.x();
        float a1b1 = a.y() * b.y();
        float a2b2 = a.z() * b.z();
        Eigen::Matrix3f m;
        m << -a1b1 - a2b2, a.y()*b.x(),   a.z()*b.x(),
             a.x()*b.y(),   -a0b0 - a2b2, a.z()*b.y(),
             a.x()*b.z(),   a.y()*b.z(),   -a0b0 - a1b1;
        return m;
    }
}

Contact::Contact(RigidBody* b0, RigidBody* b1,
                 const Eigen::Vector3f& contactPoint,
                 const Eigen::Vector3f& normal,
                 float penetration)
 : Joint(b0, b1),
   p(contactPoint),
   n(normal),
   t(Eigen::Vector3f::Zero()),
   b(Eigen::Vector3f::Zero()),
   pene(penetration),
   relVel(Eigen::Vector3f::Zero()),
   faceIndex0(-1),
   faceIndex1(-1),
   restitution(0.0f),
   bias(Contact::baumgarte),
   persistent(false),
   prevLambda(Eigen::Vector3f::Zero()),
   k(1.0f),
   MinvJ0T(JBlockTranspose::Zero()),
   MinvJ1T(JBlockTranspose::Zero())
{
    dim = 3;
    J0.setZero(3,6);
    J1.setZero(3,6);
    J0Minv.setZero(3,6);
    J1Minv.setZero(3,6);
    lambda.setZero(3);
    phi.setZero(3);
    phi(0) = penetration;

    body0->contacts.push_back(this);
    body1->contacts.push_back(this);
}

Contact::Contact()
 : Joint(),
   p(Eigen::Vector3f::Zero()),
   n(Eigen::Vector3f::Zero()),
   t(Eigen::Vector3f::Zero()),
   b(Eigen::Vector3f::Zero()),
   pene(0.0f),
   relVel(Eigen::Vector3f::Zero()),
   faceIndex0(-1),
   faceIndex1(-1),
   restitution(0.0f),
   bias(Contact::baumgarte),
   persistent(false),
   prevLambda(Eigen::Vector3f::Zero()),
   k(1.0f),
   MinvJ0T(JBlockTranspose::Zero()),
   MinvJ1T(JBlockTranspose::Zero())
{
    dim = 3;
    lambda.setZero(3);
    phi.setZero(3);
}

Contact::~Contact() = default;

void Contact::setFaceIndices(int face0, int face1) {
    faceIndex0 = face0;
    faceIndex1 = face1;

    // Record face hits
    if (faceIndex0 >= 0) {
        FaceContactTracker::recordHit(body0, faceIndex0);
    }
    if (faceIndex1 >= 0) {
        FaceContactTracker::recordHit(body1, faceIndex1);
    }
}

void Contact::computeContactFrame()
{
    t = n.cross(Eigen::Vector3f(1, 0, 0));
    if (t.norm() < 1e-5f)
        t = n.cross(Eigen::Vector3f(0, 1, 0));
    t.normalize();

    b = n.cross(t);
    b.normalize();
}

void Contact::computeJacobian()
{
    const Eigen::Vector3f rr0 = p - body0->x;
    const Eigen::Vector3f rr1 = p - body1->x;

    J0.setZero();
    J1.setZero();
    J0Minv.setZero();
    J1Minv.setZero();
    lambda.setZero();
    phi.setZero();
    phi(0) = pene;

    // Normal constraint
    J0.block<1,3>(0,0) = n.transpose();
    J0.block<1,3>(0,3) = rr0.cross(n).transpose();
    J1.block<1,3>(0,0) = -n.transpose();
    J1.block<1,3>(0,3) = -rr1.cross(n).transpose();

    // Friction tangents
    J0.block<1,3>(1,0) = t.transpose();
    J0.block<1,3>(1,3) = rr0.cross(t).transpose();
    J1.block<1,3>(1,0) = -t.transpose();
    J1.block<1,3>(1,3) = -rr1.cross(t).transpose();

    J0.block<1,3>(2,0) = b.transpose();
    J0.block<1,3>(2,3) = rr0.cross(b).transpose();
    J1.block<1,3>(2,0) = -b.transpose();
    J1.block<1,3>(2,3) = -rr1.cross(b).transpose();

    // Compute J * M^-1 blocks
    J0Minv.block<3,3>(0,0) = (1.0f/body0->mass) * J0.block<3,3>(0,0);
    J0Minv.block<3,3>(0,3) = J0.block<3,3>(0,3) * body0->Iinv;
    J1Minv.block<3,3>(0,0) = (1.0f/body1->mass) * J1.block<3,3>(0,0);
    J1Minv.block<3,3>(0,3) = J1.block<3,3>(0,3) * body1->Iinv;

    // Explicitly set these members
    MinvJ0T = J0Minv.transpose();
    MinvJ1T = J1Minv.transpose();
}

void Contact::computeGeometricStiffness()
{
    const Eigen::Vector3f rr0 = body0->q * p;
    const Eigen::Vector3f rr1 = body1->q * p;

    const Eigen::Vector3f p0 = rr0 - body0->x;
    const Eigen::Vector3f p1 = rr1 - body1->x;

    const Eigen::Vector3f nlambda = lambda[0] * n;

    G0.setZero();
    G0.block<3, 3>(3, 3) = prodOfCrossProd(nlambda, p0);
    G1.setZero();
    G1.block<3, 3>(3, 3) = -prodOfCrossProd(nlambda, p1);
}

void Contact::warmStart()
{
    lambda = prevLambda;
}

void Contact::reset()
{
    prevLambda = lambda;
    lambda.setZero();
    persistent = false;
}