#include "collision/Geometry.h"

// — Sphere —

Sphere::Sphere(float _radius)
  : radius(_radius) {}

Sphere::~Sphere() = default;

Eigen::Matrix3f Sphere::computeInertia(float mass) {
  Eigen::Matrix3f I = Eigen::Matrix3f::Zero();
  float r2 = radius * radius;
  I.diagonal().setConstant((2.0f/5.0f) * mass * r2);
  return I;
}

eGeometryType Sphere::getType() const {
  return kSphere;
}

AABB Sphere::computeAABB() const {
  Eigen::Vector3d d(radius, radius, radius);
  return AABB(-d, d);
}


// — Box —

Box::Box(const Eigen::Vector3f& _dim)
  : dim(_dim) {}

Box::~Box() = default;

Eigen::Matrix3f Box::computeInertia(float mass) {
  Eigen::Matrix3f I = Eigen::Matrix3f::Zero();
  float x2 = dim.x()*dim.x();
  float y2 = dim.y()*dim.y();
  float z2 = dim.z()*dim.z();
  I(0,0) = (mass/12.0f) * (y2 + z2);
  I(1,1) = (mass/12.0f) * (x2 + z2);
  I(2,2) = (mass/12.0f) * (x2 + y2);
  return I;
}

eGeometryType Box::getType() const {
  return kBox;
}

AABB Box::computeAABB() const {
  Eigen::Vector3d half(dim.x()/2.0, dim.y()/2.0, dim.z()/2.0);
  return AABB(-half, half);
}


// — Cylinder —

Cylinder::Cylinder(float _height, float _radius)
  : height(_height), radius(_radius) {}

Cylinder::~Cylinder() = default;

Eigen::Matrix3f Cylinder::computeInertia(float mass) {
  Eigen::Matrix3f I = Eigen::Matrix3f::Zero();
  float h2 = height*height;
  float r2 = radius*radius;
  float s  = 1.0f/12.0f;
  I(0,0) = s * mass * (3.0f*r2 + h2);
  I(1,1) = 0.5f * mass * r2;
  I(2,2) = s * mass * (3.0f*r2 + h2);
  return I;
}

eGeometryType Cylinder::getType() const {
  return kCylinder;
}

AABB Cylinder::computeAABB() const {
  Eigen::Vector3d half(radius, height/2.0, radius);
  return AABB(-half, half);
}


// — Plane —

Plane::Plane(const Eigen::Vector3f& _p, const Eigen::Vector3f& _n)
  : p(_p), n(_n) {}

Plane::~Plane() = default;

Eigen::Matrix3f Plane::computeInertia(float /*mass*/) {
  return Eigen::Matrix3f::Zero();
}

eGeometryType Plane::getType() const {
  return kPlane;
}

AABB Plane::computeAABB() const {
  Eigen::Vector3d pt(p.x(), p.y(), p.z());
  return AABB(pt, pt);
}
