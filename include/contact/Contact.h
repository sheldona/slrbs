#pragma once

#include "joint/Joint.h"
#include <Eigen/Dense>

class RigidBody;

// Contact class: models frictional contact constraint with a boxed LCP
class Contact : public Joint
{
public:
    // Enum to control the contact model type
    enum class ContactModel {
        COULOMB,        // Standard Coulomb friction model
        BOX,            // Box model approximation (default)
        CONE            // Cone model with constraint manifold
    };

    // Default constructor
    Contact();

    // Main constructor
    Contact(RigidBody* b0, RigidBody* b1,
            const Eigen::Vector3f& contactPoint,
            const Eigen::Vector3f& normal,
            float penetration);

    // Destructor
    virtual ~Contact();

    // Implement required Joint virtual functions
    virtual eConstraintType getType() const override { return kContact; }
    virtual std::string getTypeName() const override { return "Contact"; }

    // Set face indices for contact visualization after construction
    void setFaceIndices(int face0, int face1);

    // Compute contact frame (tangent, bitangent)
    void computeContactFrame();

    // Compute Jacobian for this contact
    virtual void computeJacobian() override;

    // Compute geometric stiffness
    virtual void computeGeometricStiffness() override;

    // Warm start constraint (use previous lambda)
    void warmStart();

    // Reset constraint state
    void reset();

    // Contact state
    Eigen::Vector3f p;                // Contact point location
    Eigen::Vector3f n;                // Contact normal
    Eigen::Vector3f t;                // Contact tangent
    Eigen::Vector3f b;                // Contact bitangent
    float pene;                       // Penetration depth
    Eigen::Vector3f relVel;           // Relative velocity at contact

    // Face indices for visualization
    int faceIndex0;                   // Face index on body0 (-1 if not applicable)
    int faceIndex1;                   // Face index on body1 (-1 if not applicable)

    // Contact parameters
    float restitution;                // Coefficient of restitution
    float bias;                       // Baumgarte stabilization factor
    bool persistent;                  // Persistent contact flag

    // Cached values
    Eigen::Vector3f prevLambda;       // Previous impulse (for warm starting)
    float k;                          // Constraint stiffness

    // Static members for contact parameters
    static float mu;                  // Friction coefficient
    static float restitutionThreshold; // Velocity threshold for restitution
    static float baumgarte;           // Baumgarte stabilization factor
    static float slop;                // Contact slop factor
    static ContactModel model;        // Contact model type
    static bool warmStarting;         // Enable warm starting
    static float warmStartFactor;     // Factor for warm starting (0-1)
    static float maxPenetration;      // Maximum allowed penetration
    static float frictionTangentScale; // Scale factor for friction tangent
    static bool useEnhancedFriction;  // Use enhanced friction model

    using JBlock          = Eigen::Matrix<float,3,6>;
    using JBlockTranspose = Eigen::Matrix<float,6,3>;
    JBlockTranspose MinvJ0T, MinvJ1T;
};