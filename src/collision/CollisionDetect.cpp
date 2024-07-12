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

}


CollisionDetect::CollisionDetect(RigidBodySystem* rigidBodySystem) : m_rigidBodySystem(rigidBodySystem)
{

}

void CollisionDetect::detectCollisions()
{
    // Next, loop over all pairs of bodies and test for contacts.
    //
    auto bodies = m_rigidBodySystem->getBodies();
    for(unsigned int i = 0; i < bodies.size(); ++i)
    {
        for(unsigned int j = i+1; j < bodies.size(); ++j)
        {
            RigidBody* body0 = bodies[i];
            RigidBody* body1 = bodies[j];

            // Special case: skip tests for pairs of static bodies.
            //
            if (body0->fixed && body1->fixed) 
                continue;

            // Test for sphere-sphere collision.
            if( body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kSphere )
            {
                collisionDetectSphereSphere(body0, body1);
            }
            // Test for sphere-box collision
            else if( body0->geometry->getType() == kSphere &&
                     body1->geometry->getType() == kBox )
            {
                collisionDetectSphereBox(body0, body1);
            }
            // Test for box-sphere collision (order swap)
            else if( body1->geometry->getType() == kSphere &&
                     body0->geometry->getType() == kBox )
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
            else if (body1->geometry->getType() == kBox &&
                body0->geometry->getType() == kBox)
            {
				collisionDetectBoxBox(body0, body1);
            }
        }
    }
}

void CollisionDetect::computeContactJacobians()
{
    for(auto c : m_contacts)
    {
        c->computeContactFrame();
        c->computeJacobian();
    }
}

void CollisionDetect::clear()
{
    for(auto c : m_contacts)
    {
        delete c;
    }
    m_contacts.clear();

    auto bodies = m_rigidBodySystem->getBodies();
    for(auto b : bodies)
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
    if( dist < rsum )
    {
        const Eigen::Vector3f n = vec / dist;
        const Eigen::Vector3f p = 0.5f * ((body0->x - sphere0->radius*n) + (body1->x + sphere1->radius*n));
        const float phi = dist-rsum;

        m_contacts.push_back( new Contact(body0, body1, p, n, phi) );
    }
}

void CollisionDetect::collisionDetectSphereBox(RigidBody* body0, RigidBody* body1)
{
    Sphere* sphere = dynamic_cast<Sphere*>(body0->geometry.get());
    Box* box = dynamic_cast<Box*>(body1->geometry.get());

    const Eigen::Vector3f clocal = body1->q.inverse() * (body0->x - body1->x);

    Eigen::Vector3f q(0,0,0);
    for(unsigned int i = 0; i < 3; ++i)
    {
        //q[i] = std::max(-box->dim[i]/2.0f, std::min(box->dim[i]/2.0f, clocal[i]));
        q[i] = clocal[i];
        if (q[i] < (-box->dim[i] / 2.0f)) q[i] = -box->dim[i] / 2.0f;
        else if (q[i] > (box->dim[i] / 2.0f)) q[i] = (box->dim[i] / 2.0f);
    }

    const Eigen::Vector3f dx = clocal - q;
    const float dist = dx.norm();
    if( dist < sphere->radius )
    {
        const Eigen::Vector3f n = body1->q * (dx/dist);
        const Eigen::Vector3f p = body1->q * q + body1->x;
        const float phi = dist - sphere->radius;

        m_contacts.push_back( new Contact(body0, body1, p, n, phi) );
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

    if ( std::fabs(dp) > 0.995f) // aligned with plane normal
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
        if ( std::fabs(w.dot(Eigen::Vector3f(1,0,0))) > 0.01f)
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

        const Eigen::Vector3f a = body0->x + float(0.5f) * cyl->height * w;
        const float dist = distancePointPlane(a, planep, planen);
        if (dist < 0.01f)
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(a + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(a - cyl->radius * u, planep, planen);
            float phiC = distancePointPlane(a + cyl->radius * v, planep, planen);
            float phiD = distancePointPlane(a - cyl->radius * v, planep, planen);

            if (phiA < 0.0f) {
                m_contacts.push_back(new Contact(body0, body1, a + cyl->radius * u, n, phiA));
            }
            if (phiB < 0.0f) {
                m_contacts.push_back(new Contact(body0, body1, a - cyl->radius * u, n, phiB));
            }
            if (phiC < 0.0f) {
                m_contacts.push_back(new Contact(body0, body1, a + cyl->radius * v, n, phiC));
            }
            if (phiD < 0.0f) {
                m_contacts.push_back(new Contact(body0, body1, a - cyl->radius * v, n, phiD));
            }
        }

    }
    else if ( std::fabs(dp) < 0.005f)  // penpendicular to plane
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
        if (dist < 0.01f )
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            if (phiA < 0.0f) {
                m_contacts.push_back(new Contact(body0, body1, cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, phiA));
            }
            if (phiB < 0.0f) {
                m_contacts.push_back(new Contact(body0, body1, cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, phiB));
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
        if (dist < 0.0f)
        {
            const Eigen::Vector3f n = planen;
            const Eigen::Vector3f p = a;
            float phi = dist;
            m_contacts.push_back(new Contact(body0, body1, p, n, phi));
        }
    }
    
}

void CollisionDetect::collisionDetectBoxBox(RigidBody* body0, RigidBody* body1)
{
    Box* box1 = dynamic_cast<Box*>(body0->geometry.get());
    Box* box2 = dynamic_cast<Box*>(body1->geometry.get());

    // Helper function to compute the vertices of a box
    auto getVertices = [](const Box& box, Eigen::Vector3f& center, Eigen::Matrix3f& R) {
        Eigen::Vector3f hs = box.dim * 0.5f;
        
        std::vector<Eigen::Vector3f> vertices(8);
        vertices[0] = center + R * Eigen::Vector3f(hs.x(), hs.y(), hs.z());
        vertices[1] = center + R * Eigen::Vector3f(-hs.x(), hs.y(), hs.z());
        vertices[2] = center + R * Eigen::Vector3f(hs.x(), -hs.y(), hs.z());
        vertices[3] = center + R * Eigen::Vector3f(hs.x(), hs.y(), -hs.z());
        vertices[4] = center + R * Eigen::Vector3f(-hs.x(), -hs.y(), hs.z());
        vertices[5] = center + R * Eigen::Vector3f(hs.x(), -hs.y(), -hs.z());
        vertices[6] = center + R * Eigen::Vector3f(-hs.x(), hs.y(), -hs.z());
        vertices[7] = center + R * Eigen::Vector3f(-hs.x(), -hs.y(), -hs.z());
        return vertices;
    };

    std::vector<Eigen::Vector3f> axes;
    axes.reserve(15);
    Eigen::Matrix3f R1 = body0->q.toRotationMatrix();
    Eigen::Matrix3f R2 = body1->q.toRotationMatrix();

    // Add face normals of both boxes
    axes.push_back(R1.col(0));
    axes.push_back(R1.col(1));
    axes.push_back(R1.col(2));
    axes.push_back(R2.col(0));
    axes.push_back(R2.col(1));
    axes.push_back(R2.col(2));

    // Add cross products of edge directions
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            axes.push_back(R1.col(i).cross(R2.col(j)));
        }
    }

    // Check for separation on each axis
    float minPenetrationDepth = std::numeric_limits<float>::max();
    Eigen::Vector3f bestAxis;
    bool collision = true;

    for (const auto& axis : axes) {
        Eigen::Vector3f axisNorm = axis.normalized();
        if (axisNorm.squaredNorm() < 0.0001f)
            continue;

        // Project the vertices of both boxes onto the axis
        auto project = [&axisNorm](const std::vector<Eigen::Vector3f>& vertices) {
            float minProj = std::numeric_limits<float>::max();
            float maxProj = -std::numeric_limits<float>::max();
            for (const auto& vertex : vertices) {
                float proj = vertex.dot(axisNorm);
                if (proj < minProj) minProj = proj;
                if (proj > maxProj) maxProj = proj;
            }
            return std::make_pair(minProj, maxProj);
        };

        auto vertices1 = getVertices(*box1, body0->x, R1);
        auto vertices2 = getVertices(*box2, body1->x, R2);

        auto [min1, max1] = project(vertices1);
        auto [min2, max2] = project(vertices2);

        if (max1 < min2 || max2 < min1) {
            collision = false;
            break;
        }
        else {
            float overlap = std::min(max1, max2) - std::max(min1, min2);
            if (overlap < minPenetrationDepth) {
                minPenetrationDepth = overlap;
                bestAxis = axisNorm;
            }
        }
    }

    if (!collision || bestAxis.squaredNorm() < 0.0001f) 
        return;

    // Compute contact points if collision is detected
    Eigen::Vector3f contactNormal = bestAxis;
    float penetrationDepth = minPenetrationDepth;

    // Find the point of deepest penetration
    for (const auto& vertex : getVertices(*box2, body1->x, R2)) {
        double proj = vertex.dot(contactNormal);
        if (proj <= penetrationDepth) {
            Contact* cp = new Contact(body1, body0, vertex, contactNormal, -penetrationDepth);
            m_contacts.push_back(cp);
        }
    }

    for (const auto& vertex : getVertices(*box1, body0->x, R1)) {
        double proj = vertex.dot(contactNormal);
        if (proj <= penetrationDepth) {
            Contact* cp = new Contact(body0, body1, vertex, -contactNormal, -penetrationDepth);
            m_contacts.push_back(cp);
        }
    }
}