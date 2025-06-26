#include "collision/CollisionDetect.h"

#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"


namespace
{
    // Compute the distance from a point to a plane defined by point and normal pair.
    // If the point is "inside" the plane, the returned distance is negative.
    static inline float distancePointPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& plane_p, const Eigen::Vector3f& plane_n)
    {
        const Eigen::Vector3f v = (p - plane_p);
        const float d = v.dot(plane_n);
        return d;
    }

    static const float eps = 1e-2f;     // Collision "thickness"
}

CollisionDetect::CollisionDetect() : m_rigidBodySystem(nullptr), m_maxContacts(0) { }

CollisionDetect::CollisionDetect(RigidBodySystem* rigidBodySystem) : m_rigidBodySystem(rigidBodySystem), m_maxContacts(1024)
{
    m_contacts.reserve(m_maxContacts);
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
            else if (body0->geometry->getType() == kBox &&
                body1->geometry->getType() == kPlane)
            {
                collisionDetectBoxPlane(body0, body1);
            }
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
    for (Contact& c : m_contacts)
    {
        c.computeJacobian();
    }
}

void CollisionDetect::clear()
{
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
    if (dist < (rsum + eps))
    {
        const Eigen::Vector3f n = vec / dist;
        const Eigen::Vector3f p = 0.5f * ((body0->x - sphere0->radius * n) + (body1->x + sphere1->radius * n));
        const float phi = std::min(0.0f, dist - rsum);

        m_contacts.emplace_back(body0, body1, p, n, phi);
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
    if (dist < (sphere->radius + eps))
    {
        const Eigen::Vector3f n = body1->q * (dx / dist);
        const Eigen::Vector3f p = body1->q * q + body1->x;
        const float phi = std::min(0.0f, dist - sphere->radius);

        m_contacts.emplace_back(body0, body1, p, n, phi);
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

    if (std::fabs(dp) > 0.99f) // aligned with plane normal
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
        if (dist < 1e-3f)
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(a + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(a - cyl->radius * u, planep, planen);
            float phiC = distancePointPlane(a + cyl->radius * v, planep, planen);
            float phiD = distancePointPlane(a - cyl->radius * v, planep, planen);

            if (phiA < eps) {
                m_contacts.emplace_back(body0, body1, a + cyl->radius * u, n, std::min(0.0f, phiA));
            }
            if (phiB < eps) {
                m_contacts.emplace_back(body0, body1, a - cyl->radius * u, n, std::min(0.0f, phiB));
            }
            if (phiC < eps) {
                m_contacts.emplace_back(body0, body1, a + cyl->radius * v, n, std::min(0.0f, phiC));
            }
            if (phiD < eps) {
                m_contacts.emplace_back(body0, body1, a - cyl->radius * v, n, std::min(0.0f, phiD));
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
        if (dist < 0.01f)
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            if (phiA < eps) {
                m_contacts.emplace_back(body0, body1, cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, std::min(0.0f, phiA));
            }
            if (phiB < eps) {
                m_contacts.emplace_back(body0, body1, cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, std::min(0.0f, phiB));
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
        if (dist < 1e-3f)
        {
            const Eigen::Vector3f n = planen;
            const Eigen::Vector3f p = a;
            const float phi = std::min(0.0f, dist);
            m_contacts.emplace_back(body0, body1, p, n, phi);
        }
    }

}

void CollisionDetect::collisionDetectBoxPlane(RigidBody* body0, RigidBody* body1)
{
    Box* box = dynamic_cast<Box*>(body0->geometry.get());
    Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());

    const std::vector<Eigen::Vector3f> pts = {
        0.5f * Eigen::Vector3f(-box->dim[0], -box->dim[1], -box->dim[2]),
        0.5f * Eigen::Vector3f(-box->dim[0], -box->dim[1], box->dim[2]),
        0.5f * Eigen::Vector3f(-box->dim[0], box->dim[1], -box->dim[2]),
        0.5f * Eigen::Vector3f(-box->dim[0], box->dim[1], box->dim[2]),
        0.5f * Eigen::Vector3f(box->dim[0], -box->dim[1], -box->dim[2]),
        0.5f * Eigen::Vector3f(box->dim[0], -box->dim[1], box->dim[2]),
        0.5f * Eigen::Vector3f(box->dim[0], box->dim[1], -box->dim[2]),
        0.5f * Eigen::Vector3f(box->dim[0], box->dim[1], box->dim[2])
    };

    // world space plane position and normal
    const Eigen::Vector3f pplane = body1->q * plane->p + body1->x;
    const Eigen::Vector3f nplane = (body1->q * plane->n).normalized();

    for (const Eigen::Vector3f& p : pts)
    {
        const Eigen::Vector3f pworld = body0->q * p + body0->x;
        const Eigen::Vector3f v = (pworld - pplane);
        const float d = v.dot(nplane);

        if (d < eps)
        {
            const float phi = std::min(0.0f, d);
            m_contacts.emplace_back(body0, body1, pworld, nplane, phi);
        }
    }

}
