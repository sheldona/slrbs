#pragma once

#ifndef COLLISION_AABB_H
#define COLLISION_AABB_H

#include <Eigen/Dense>
#include <vector>

// Axis-Aligned Bounding Box in double precision
class AABB {
public:
    AABB();
    AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

    // Compute AABB that encloses a list of points
    static AABB fromVertices(const std::vector<Eigen::Vector3d>& vertices);

    // Expand to include a point
    void expand(const Eigen::Vector3d& point);

    // Test overlap against another AABB
    bool overlaps(const AABB& other) const;

    // Center and half-extents
    Eigen::Vector3d getCenter() const;
    Eigen::Vector3d getHalfExtents() const;

    // Transform by rotation and translation
    AABB transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) const;

    Eigen::Vector3d min;
    Eigen::Vector3d max;
};

#endif // COLLISION_AABB_H