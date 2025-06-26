#pragma once

#include "joint/Joint.h"

#include <Eigen/Dense>

class RigidBody;

// Contact constraint with box friction.
// 
// This class stores the contact normal @a n and contact point @a p,
// as well as the Jacobians @a J0 and @a J1 for each body,
// which are computed by computeJacobian().
// 
// Note: contacts are considered a type of joint, but are a special case.
//
class Contact : public Joint
{
public: 
    static float mu_r;            // Mean coefficient of friction (global)
    static float sigma_r;         // Friction variance

    enum eContactStateType { kStick, kSlip };
public:

    // Constructor with all parameters.
    Contact(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _p, const Eigen::Vector3f& _n, float _phi);

    virtual ~Contact();

    virtual eConstraintType getType() const override { return kContact; }

    float mu;
    Eigen::Vector3f p;          // The contact point.
    Eigen::Vector3f n;          // The contact normal.
    Eigen::Vector3f t, b;       // Tangent directions.
    float pene;                 // Penetration
    eContactStateType state;    // stick or slip?
    bool matched;    // contact was matched to previous frame

    // Computes a contact frame using the contact normal @a n
    // and the provided direction @a dir, which is aligned with the first tangent direction.
    // The resulting frame is stored in the basis vectors @a n, @a t1, and @a t2.
    // Also populates J0, J1 and related matrices.
    //
    virtual void computeJacobian();

protected:

    // Default constructor (hidden)
    explicit Contact();
};

struct ContactState
{
    float mu;
    Eigen::Vector3f p[2];
    Eigen::Vector3f p_pred[2];
    Eigen::Vector3f n[2];
    RigidBody* body[2];
    float pene;
    Contact::eContactStateType state;
    float lambda[3];
};
