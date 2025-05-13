#include "joint/Distance.h"
#include "rigidbody/RigidBody.h"
#include <Eigen/Dense>
#include <iostream> // Include for debugging
#include <cassert> // Include for assertions

Distance::Distance() : Joint()
{
    // Default constructor
}

Distance::Distance(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, float _d)
    : Joint(_body0, _body1, _r0, Eigen::Quaternionf::Identity(), _r1, Eigen::Quaternionf::Identity(), kDistance), d(_d)
{
    // Initialize the dimensions and matrices for the distance joint
    dim = 1;
    J0.setZero(1, 6);
    J1.setZero(1, 6);
    J0Minv.setZero(1, 6);
    J1Minv.setZero(1, 6);
    phi.setZero(1);
    lambda.setZero(1);
}

void Distance::computeJacobian()
{
    // Compute the relative position vectors
    Eigen::Vector3f p0 = body0->x + body0->q * r0;
    Eigen::Vector3f p1 = body1->x + body1->q * r1;

    // Compute the constraint error phi
    Eigen::Vector3f diff = p1 - p0;
    float dist = diff.norm();
    phi[0] = dist - d;

    // Normalize the difference vector to get the constraint direction
    Eigen::Vector3f n = diff.normalized();

    // Compute the Jacobians J0 and J1
    J0.block<1, 3>(0, 0) = -n.transpose();
    J0.block<1, 3>(0, 3) = -(body0->q * r0).cross(n).transpose();

    J1.block<1, 3>(0, 0) = n.transpose();
    J1.block<1, 3>(0, 3) = (body1->q * r1).cross(n).transpose();

    // Precompute the terms J0Minv and J1Minv
    J0Minv.block<1, 3>(0, 0) = J0.block<1, 3>(0, 0) * (1.0f / body0->mass);
    J0Minv.block<1, 3>(0, 3) = J0.block<1, 3>(0, 3) * body0->Iinv;

    J1Minv.block<1, 3>(0, 0) = J1.block<1, 3>(0, 0) * (1.0f / body1->mass);
    J1Minv.block<1, 3>(0, 3) = J1.block<1, 3>(0, 3) * body1->Iinv;
}