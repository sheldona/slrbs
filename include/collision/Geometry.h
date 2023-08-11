#pragma once

#include "util/Types.h"
#include <Eigen/Dense>

// List of geometry type ids.
enum eGeometryType { kSphere, kBox, kPlane, kCylinder };

// Generic geometry interface.
//
class Geometry
{
public:
    virtual Eigen::Matrix3f computeInertia(float _mass) = 0;

    virtual eGeometryType getType() const  = 0;

protected:
    Eigen::Matrix3f m_I;          // Inertia 3x3 matrix for this. Only used for local computations. (internal)
};


// Sphere geometry.
//
class Sphere : public Geometry
{
public:
    float radius;           // Sphere radius.

    Sphere(float _radius) : radius(_radius) {}
    virtual ~Sphere() {}

    virtual Eigen::Matrix3f computeInertia(float _mass) override
    {
        m_I.setZero();
        m_I(0,0) = m_I(1,1) = m_I(2,2) = (2.0f/5.0f) * _mass * radius * radius;
        return m_I;
    }

    virtual eGeometryType getType() const override { return kSphere; }

};

// Box geometry.
//
class Box : public Geometry
{
public:
    Eigen::Vector3f dim;        // Box dimensions.

    Box(const Eigen::Vector3f& _dim) : dim(_dim) {

    }
    virtual ~Box() {}

    virtual Eigen::Matrix3f computeInertia(float _mass) override
    {
        m_I.setZero();
        m_I(0,0) = (1.0f/12.0f)*_mass*(dim[1]*dim[1] + dim[2]*dim[2]);
        m_I(1,1) = (1.0f/12.0f)*_mass*(dim[0]*dim[0] + dim[2]*dim[2]);
        m_I(2,2) = (1.0f/12.0f)*_mass*(dim[0]*dim[0] + dim[1]*dim[1]);
        return m_I;
    }

    virtual eGeometryType getType() const override { return kBox; }

};


// Cylinder geometry.
//
class Cylinder : public Geometry
{
public:
    float height, radius;        // Cylinder height and radius

    Cylinder(float _height, float _radius) : height(_height), radius(_radius) {

    }
    virtual ~Cylinder() {}

    virtual Eigen::Matrix3f computeInertia(float _mass) override
    {
        static const float s = 1.0f / 12.0f;
        const float h2 = height * height;
        const float r2 = radius * radius;
        const float r2_h2_3 = (3.0f * r2 + h2);
        m_I.setZero();
        m_I(0, 0) = s * _mass * r2_h2_3;
        m_I(1, 1) = 0.5f * _mass * r2;
        m_I(2, 2) = s * _mass * r2_h2_3;
        return m_I;
    }

    virtual eGeometryType getType() const override { return kCylinder; }

};

// Plane geometry.
//
class Plane : public Geometry
{
public:
    Eigen::Vector3f p, n;        // Plane point and normal.

    Plane(const Eigen::Vector3f& _p, const Eigen::Vector3f& _n) : p(_p), n(_n) {

    }
    virtual ~Plane() {}

    virtual Eigen::Matrix3f computeInertia(float _mass) override
    {
        m_I.setZero();  // ignore inertia, should only be used for static objects.
        return m_I;
    }

    virtual eGeometryType getType() const override { return kPlane; }

};
