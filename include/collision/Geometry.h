#pragma once

#include <Eigen/Dense>
#include "collision/AABB.h"

// Geometry type identifiers
enum eGeometryType { kSphere, kBox, kPlane, kCylinder };

/// Abstract base for all collision‐shape primitives
class Geometry {
public:
    virtual ~Geometry() = default;

    /// Compute this shape's local inertia tensor for a given mass
    virtual Eigen::Matrix3f computeInertia(float mass) = 0;

    /// Identify which concrete subclass this is
    virtual eGeometryType getType() const = 0;

    /// Compute the axis‐aligned bounding box (in the shape's local frame)
    virtual AABB computeAABB() const = 0;
};

/// Sphere primitive
class Sphere : public Geometry {
public:
    explicit Sphere(float radius);
    ~Sphere() override;

    Eigen::Matrix3f computeInertia(float mass) override;
    eGeometryType getType() const override;
    AABB computeAABB() const override;

    float radius;
};

/// Box primitive (dimensions = full width/height/depth)
class Box : public Geometry {
public:
    explicit Box(const Eigen::Vector3f& dim);
    ~Box() override;

    Eigen::Matrix3f computeInertia(float mass) override;
    eGeometryType getType() const override;
    AABB computeAABB() const override;

    Eigen::Vector3f dim;
};

/// Cylinder primitive (axis = local Y)
class Cylinder : public Geometry {
public:
    Cylinder(float height, float radius);
    ~Cylinder() override;

    Eigen::Matrix3f computeInertia(float mass) override;
    eGeometryType getType() const override;
    AABB computeAABB() const override;

    float height;
    float radius;
};

/// Plane primitive (modeled as infinite, but AABB collapsed to a point)
class Plane : public Geometry {
public:
    Plane(const Eigen::Vector3f& point, const Eigen::Vector3f& normal);
    ~Plane() override;

    Eigen::Matrix3f computeInertia(float mass) override;
    eGeometryType getType() const override;
    AABB computeAABB() const override;

    Eigen::Vector3f p;
    Eigen::Vector3f n;
};
