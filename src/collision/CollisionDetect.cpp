#include "collision/CollisionDetect.h"

#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionDispatch/btBoxBoxDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

namespace
{

    // Collision margin 
    static const float margin = 1e-2f;

    // Compute the distance from a point to a plane defined by point and normal pair.
    // If the point is "inside" the plane, the returned distance is negative.
    static inline float distancePointPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& plane_p, const Eigen::Vector3f& plane_n)
    {
        const Eigen::Vector3f v = (p - plane_p);
        const float d = v.dot(plane_n);
        return d;
    }


    // Plane-vertex collision test.
    static inline bool collisionDetectPointPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& plane_p, const Eigen::Vector3f& plane_n, float& pene)
    {
        const float dp = (p - plane_p).dot(plane_n);
        if (dp < margin)
        {
            pene = std::min(0.0f, dp);
            return true;
        }
        return false;
    }

}


CollisionDetect::CollisionDetect(RigidBodySystem* rigidBodySystem) : m_rigidBodySystem(rigidBodySystem)
{

}

void CollisionDetect::detectCollisions()
{
    // Next, loop over all pairs of bodies and test for contacts.
    //
    auto bodies = m_rigidBodySystem->getBodies();
    for (unsigned int i = 0; i < bodies.size(); ++i)
    {
        for (unsigned int j = i + 1; j < bodies.size(); ++j)
        {
            RigidBody* body0 = bodies[i];
            RigidBody* body1 = bodies[j];

            // Special case: skip tests for pairs of static bodies.
            //
            if (body0->fixed && body1->fixed)
                continue;

            // Test for sphere-sphere collision.
            if (body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kSphere)
            {
                collisionDetectSphereSphere(body0, body1);
            }
            // Test for sphere-box collision
            else if (body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kBox)
            {
                collisionDetectSphereBox(body0, body1);
            }
            // Test for box-sphere collision (order swap)
            else if (body1->geometry->getType() == kSphere &&
                body0->geometry->getType() == kBox)
            {
                collisionDetectSphereBox(body1, body0);
            }
            // Test for cylinder-plane collision
            else if (body0->geometry->getType() == kCylinder &&
                body1->geometry->getType() == kPlane)
            {
                collisionDetectCylinderPlane(body0, body1);
            }
            // Test for cylinder-plane collision
            else if (body1->geometry->getType() == kCylinder &&
                body0->geometry->getType() == kPlane)
            {
                collisionDetectCylinderPlane(body1, body0);
            }
            // Test for box-box collision
            else if (body0->geometry->getType() == kBox &&
                body1->geometry->getType() == kBox)
            {
                collisionDetectBoxBox(body0, body1);
            }
            // Test for box-plane collision
            else if (body0->geometry->getType() == kBox &&
                body1->geometry->getType() == kPlane)
            {
                collisionDetectBoxPlane(body0, body1);
            }
            // Test for box-plane collision
            else if (body1->geometry->getType() == kBox &&
                body0->geometry->getType() == kPlane)
            {
                collisionDetectBoxPlane(body1, body0);
            }

        }
    }
}

void CollisionDetect::computeContactJacobians()
{
    for (auto c : m_contacts)
    {
        c->computeContactFrame();
        c->computeJacobian();
    }
}

void CollisionDetect::clear()
{
    for (auto c : m_contacts)
    {
        delete c;
    }
    m_contacts.clear();

    auto bodies = m_rigidBodySystem->getBodies();
    for (auto b : bodies)
    {
        b->contacts.clear();
    }
}

void CollisionDetect::collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1)
{
    Sphere* sphere0 = dynamic_cast<Sphere*>(body0->geometry.get());
    Sphere* sphere1 = dynamic_cast<Sphere*>(body1->geometry.get());

    // Implement sphere-sphere collision detection.
    // The function should check if a collision exists, and if it does
    // compute the contact normal, contact point, and penetration depth.
    //
    Eigen::Vector3f vec = body0->x - body1->x;

    const float rsum = (sphere0->radius + sphere1->radius);
    const float dist = vec.norm();
    if (dist < (rsum + 1e-3f))
    {
        const Eigen::Vector3f n = vec / dist;
        const Eigen::Vector3f p = 0.5f * ((body0->x - sphere0->radius * n) + (body1->x + sphere1->radius * n));
        const float phi = std::min(0.0f, dist - rsum);

        m_contacts.push_back(new Contact(body0, body1, p, n, phi));
    }
}

void CollisionDetect::collisionDetectSphereBox(RigidBody* body0, RigidBody* body1)
{
    Sphere* sphere = dynamic_cast<Sphere*>(body0->geometry.get());
    Box* box = dynamic_cast<Box*>(body1->geometry.get());

    const Eigen::Vector3f clocal = body1->q.inverse() * (body0->x - body1->x);

    Eigen::Vector3f q(0, 0, 0);
    for (unsigned int i = 0; i < 3; ++i)
    {
        q[i] = clocal[i];
        if (q[i] < (-box->dim[i] / 2.0f)) q[i] = -box->dim[i] / 2.0f;
        else if (q[i] > (box->dim[i] / 2.0f)) q[i] = (box->dim[i] / 2.0f);
    }

    const Eigen::Vector3f dx = clocal - q;
    const float dist = dx.norm();
    if (dist < (sphere->radius + 1e-3f))
    {
        const Eigen::Vector3f n = body1->q * (dx / dist);
        const Eigen::Vector3f p = body1->q * q + body1->x;
        const float phi = std::min(0.0f, dist - sphere->radius);

        m_contacts.push_back(new Contact(body0, body1, p, n, phi));
    }
}

void CollisionDetect::collisionDetectCylinderPlane(RigidBody* body0, RigidBody* body1)
{
    Cylinder* cyl = dynamic_cast<Cylinder*>(body0->geometry.get());
    Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());

    // y-axis is the principal axis
    const Eigen::Vector3f cyldir = body0->q * Eigen::Vector3f(0, 1, 0);
    const Eigen::Vector3f planen = body1->q * plane->n;
    const Eigen::Vector3f planep = body1->q * plane->p + body1->x;

    const float dp = cyldir.dot(planen);

    if (std::fabs(dp) > 0.995f) // aligned with plane normal
    {
        Eigen::Vector3f w;
        if (dp < 0.0f)
        {
            w = cyldir;
        }
        else
        {
            w = -cyldir;
        }

        Eigen::Vector3f u, v;
        if (std::fabs(w.dot(Eigen::Vector3f(1, 0, 0))) > 0.01f)
        {
            u = w.cross(Eigen::Vector3f(1, 0, 0));
        }
        else
        {
            u = w.cross(Eigen::Vector3f(0, 0, 1));
        }
        u.normalize();
        v = w.cross(u);
        v.normalize();

        const Eigen::Vector3f a = body0->x + 0.5f * cyl->height * w;
        const float dist = distancePointPlane(a, planep, planen);
        if (dist < margin)
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(a + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(a - cyl->radius * u, planep, planen);
            float phiC = distancePointPlane(a + cyl->radius * v, planep, planen);
            float phiD = distancePointPlane(a - cyl->radius * v, planep, planen);

            if (phiA < margin) {
                m_contacts.push_back(new Contact(body0, body1, a + cyl->radius * u, n, std::min(0.0f, phiA)));
            }
            if (phiB < margin) {
                m_contacts.push_back(new Contact(body0, body1, a - cyl->radius * u, n, std::min(0.0f, phiB)));
            }
            if (phiC < margin) {
                m_contacts.push_back(new Contact(body0, body1, a + cyl->radius * v, n, std::min(0.0f, phiC)));
            }
            if (phiD < margin) {
                m_contacts.push_back(new Contact(body0, body1, a - cyl->radius * v, n, std::min(0.0f, phiD)));
            }
        }

    }
    else if (std::fabs(dp) < 1e-2f)  // perpendicular to plane
    {
        const Eigen::Vector3f w = cyldir;
        Eigen::Vector3f u = (planen.cross(w)).cross(w);
        if (u.dot(planen) > 0.0f)
        {
            u = -u;
        }
        u.normalize();

        const Eigen::Vector3f cylpos = body0->x;
        const float dist = distancePointPlane(cylpos, planep, planen) - cyl->radius;
        if (dist < margin)
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            if (phiA < margin) {
                m_contacts.push_back(new Contact(body0, body1, cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, std::min(0.0f, phiA)));
            }
            if (phiB < margin) {
                m_contacts.push_back(new Contact(body0, body1, cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, std::min(0.0f, phiB)));
            }
        }
    }
    else
    {
        Eigen::Vector3f w;
        if (dp < 0.0f)
        {
            w = cyldir;
        }
        else
        {
            w = -cyldir;
        }
        w.normalize();

        Eigen::Vector3f u = (planen.cross(w)).cross(w);   // u is orthogonal to v and is oriented toward the plane
        u.normalize();

        if (u.dot(planen) > 0.0f)
        {
            u = -u;
        }
        u.normalize();

        const Eigen::Vector3f a = body0->x + float(0.5f) * cyl->height * w + cyl->radius * u;
        const float dist = distancePointPlane(a, planep, planen);
        if (dist < margin)
        {
            const Eigen::Vector3f n = planen;
            const Eigen::Vector3f p = a;
            const float phi = std::min(0.0f, dist);
            m_contacts.push_back(new Contact(body0, body1, p, n, phi));
        }
    }

}


void CollisionDetect::collisionDetectBoxBox(RigidBody* body0, RigidBody* body1)
{
    Box* box0 = dynamic_cast<Box*>(body0->geometry.get());
    Box* box1 = dynamic_cast<Box*>(body1->geometry.get());

    btBoxBoxDetector detector(box0->m_btBoxShape, box1->m_btBoxShape);

    btTransform tm0;
    tm0.setRotation(btQuaternion(body0->q.x(), body0->q.y(), body0->q.z(), body0->q.w()));
    tm0.setOrigin(btVector3(body0->x.x(), body0->x.y(), body0->x.z()));
    btTransform tm1;
    tm1.setRotation(btQuaternion(body1->q.x(), body1->q.y(), body1->q.z(), body1->q.w()));
    tm1.setOrigin(btVector3(body1->x.x(), body1->x.y(), body1->x.z()));

    btDiscreteCollisionDetectorInterface::ClosestPointInput input;
    input.m_transformA = tm0;
    input.m_transformB = tm1;

    // Custom result class to capture contact data
    struct BoxBoxResult : public btDiscreteCollisionDetectorInterface::Result
    {
        bool collision = false;
        RigidBody* body0;
        RigidBody* body1;
        std::vector<Contact*>& contacts;

        BoxBoxResult(RigidBody* _body0, RigidBody* _body1, std::vector<Contact*>& _contacts) : body0(_body0), body1(_body1), contacts(_contacts)
        {
        }

        // Called for every contact point found
        virtual void addContactPoint(const btVector3& n, const btVector3& p, btScalar depth) override
        {
            if (depth < margin)
            {   
                // Bullet uses negative depth for penetration
                collision = true;
                const float pene = std::min(0.0f, depth);
                Contact* c = new Contact(body0, body1, { p[0], p[1], p[2] }, { n[0], n[1], n[2] }, pene);
                contacts.push_back(c);
                body0->contacts.push_back(c);
                body1->contacts.push_back(c);
            }
        }

        // Unused virtuals for this specific narrow-phase test
        virtual void setShapeIdentifiersA(int partId0, int index0) override {}
        virtual void setShapeIdentifiersB(int partId1, int index1) override {}

    } result(body0, body1, m_contacts);

    detector.getClosestPoints(input, result, nullptr);
}

void CollisionDetect::collisionDetectBoxPlane(RigidBody* body0, RigidBody* body1)
{
    Box* box = dynamic_cast<Box*>(body0->geometry.get());
    Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());
    const Eigen::Vector3f pplane = body1->x;
    const Eigen::Vector3f nplane = body1->q * plane->n;
    const Eigen::Vector3f plocal[8] = {
        0.5f * Eigen::Vector3f(-box->dim(0), -box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(-box->dim(0), -box->dim(1),  box->dim(2)),
        0.5f * Eigen::Vector3f(-box->dim(0),  box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(-box->dim(0),  box->dim(1),  box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0), -box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0), -box->dim(1),  box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0),  box->dim(1), -box->dim(2)),
        0.5f * Eigen::Vector3f(box->dim(0),  box->dim(1),  box->dim(2))
    };

    for (int i = 0; i < 8; ++i)
    {
        const Eigen::Vector3f pbox = body0->q * plocal[i] + body0->x;
        float phi;
        if ( collisionDetectPointPlane(pbox, pplane, nplane, phi) )
        {
            m_contacts.push_back(new Contact(body0, body1, pbox, nplane, phi));
        }
    }
}
