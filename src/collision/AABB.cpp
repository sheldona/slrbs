#include "collision/AABB.h"
#include <limits>
#include <algorithm>

AABB::AABB() {
    min.setConstant(std::numeric_limits<double>::max());
    max.setConstant(-std::numeric_limits<double>::max());
}

AABB::AABB(const Eigen::Vector3d& min_, const Eigen::Vector3d& max_)
    : min(min_), max(max_) {}

AABB AABB::fromVertices(const std::vector<Eigen::Vector3d>& vertices) {
    AABB box;
    for (const auto& v : vertices) {
        box.expand(v);
    }
    return box;
}

void AABB::expand(const Eigen::Vector3d& point) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
}

bool AABB::overlaps(const AABB& other) const {
    return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
           (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
           (min.z() <= other.max.z() && max.z() >= other.min.z());
}

Eigen::Vector3d AABB::getCenter() const {
    return (min + max) * 0.5;
}

Eigen::Vector3d AABB::getHalfExtents() const {
    return (max - min) * 0.5;
}

AABB AABB::transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) const {
    Eigen::Vector3d newMin = translation;
    Eigen::Vector3d newMax = translation;
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d axis = rotation.col(i);
        Eigen::Vector3d c0 = axis * min[i];
        Eigen::Vector3d c1 = axis * max[i];
        for (int j = 0; j < 3; ++j) {
            double mn = std::min(c0[j], c1[j]);
            double mx = std::max(c0[j], c1[j]);
            newMin[j] += mn;
            newMax[j] += mx;
        }
    }
    return AABB(newMin, newMax);
}