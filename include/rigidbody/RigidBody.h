#pragma once

#include "collision/Geometry.h"
#include "util/Types.h"
#include <memory>
#include <vector>

class Contact;
class Joint;
struct Mesh;

namespace polyscope
{
    class SurfaceMesh;
}

// Rigid body class.
// Stores properties for rigid body simulation, including
// the geometry and list of contact constraints.
//
class RigidBody
{
public:
    RigidBody(float _mass, Geometry* _geometry, const std::string& meshFilename = "");

    RigidBody(float _mass, Geometry* _geometry, const Mesh& _mesh);

    static int counter;

    void updateInertiaMatrix();

    // Adds @a force at the specific world position @a pos. 
    void addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force);

    // Returns the rigid body linear velocity evaluated at world position @a pos
    void getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel);

    bool fixed;                         // Flag for a static rigid body. Default is 'false'.
    float mass;                         // Mass.
    Eigen::Matrix3f I, Iinv;            // Inertia and inverse inertia matrix (global)
    Eigen::Matrix3f Ibody, IbodyInv;    // Inertia and inverse inertia in the local body frame.
    Eigen::Vector3f x;                  // Position.
    Eigen::Quaternionf q;               // Orientation.

    Eigen::Vector3f xdot;               // Linear velocity.
    Eigen::Vector3f omega;              // Angular velocity.
    Eigen::Vector3f f;                  // Linear force.
    Eigen::Vector3f tau;                // Angular force (torque).

    Eigen::Vector3f fc;                 // Linear constraint force.
    Eigen::Vector3f tauc;               // Angular constraint force

    std::unique_ptr<Geometry> geometry; // The geometry of the rigid body.
    std::vector<Contact*> contacts;     // Pointer array of contact constraints involving this body.
    std::vector<Joint*> joints;         // Pointer array of joints involving this body.

    polyscope::SurfaceMesh* mesh;       // Used for rendering

    GBlock gsSum;                       // Sum of geometric stiffness blocks for this body
    Eigen::Vector3f gsDamp;             // Damping to add to the moment matrix due to geometric stiffness
};
