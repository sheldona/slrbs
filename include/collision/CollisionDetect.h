#pragma once

#include <vector>
#include <Eigen/Dense>
#include <memory>

class Contact;
class RigidBody;
class RigidBodySystem;

class CollisionDetect {
public:
    CollisionDetect(RigidBodySystem* rigidBodySystem);

    // Detect all collisions between rigid bodies in the system
    void detectCollisions();

    // Compute contact Jacobians for all detected contacts
    void computeContactJacobians();

    // Clear all contacts
    void clear();

    // Access the list of detected contacts
    const std::vector<Contact*>& getContacts() const { return m_contacts; }
    std::vector<Contact*>& getContacts() { return m_contacts; }

    void setUseOpenMP(bool enable) { m_useOpenMP = enable; }
    bool getUseOpenMP() const { return m_useOpenMP; }

private:
    // Helper method to find the closest face to a contact point
    int findFaceForContact(RigidBody* body, const Eigen::Vector3f& worldPoint);

    // Collision detection methods for different geometry combinations
    void collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1);
    void collisionDetectSphereBox(RigidBody* body0, RigidBody* body1);
    void collisionDetectCylinderPlane(RigidBody* body0, RigidBody* body1);
    void collisionDetectBoxBox(RigidBody* body0, RigidBody* body1);
    void collisionDetectCylinderBox(RigidBody* body0, RigidBody* body1);
    void collisionDetectCylinderCylinder(RigidBody* body0, RigidBody* body1);

    // The rigid body system containing all bodies
    RigidBodySystem* m_rigidBodySystem;

    // List of detected contacts
    std::vector<Contact*> m_contacts;

    // OpenMP control flag
    bool m_useOpenMP = false;
};