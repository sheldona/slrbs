#include "collision/BVH.h"
#include <algorithm>
#include <queue>
#include <numeric>
#include <polyscope/curve_network.h>
#include <limits>
#include <omp.h>

BVH::BVH(int maxDepth)
    : root(nullptr), m_maxDepth(maxDepth) {}

void BVH::build(const std::vector<glm::vec3>& vertices,
                const std::vector<glm::ivec3>& triangles) {
    std::vector<size_t> prims(triangles.size());
    std::iota(prims.begin(), prims.end(), 0);
    root = buildRecursive(prims, vertices, triangles, 0);
}

std::unique_ptr<BVHNode> BVH::buildRecursive(
    const std::vector<size_t>& prims,
    const std::vector<glm::vec3>& vertices,
    const std::vector<std::array<size_t,3>>& triangles,
    int depth) {
    // convert and delegate
    auto triI = convertFacesToIVec3(triangles);
    return buildRecursive(prims, vertices, triI, depth);
}

std::unique_ptr<BVHNode> BVH::buildRecursive(
    const std::vector<size_t>& prims,
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::ivec3>& tris,
    int depth) {
    auto node = std::make_unique<BVHNode>();
    // compute node bounds
    AABB box;
    for (auto idx : prims) {
        auto b = computeTriangleBounds(
            vertices[tris[idx].x],
            vertices[tris[idx].y],
            vertices[tris[idx].z]
        );
        box.expand(b.min);
        box.expand(b.max);
    }
    node->bounds = box;

    // leaf?
    if (depth >= m_maxDepth || prims.size() <= 4) {
        node->primitiveIndices = prims;
        return node;
    }

    // choose split axis
    Eigen::Vector3d ext = box.getHalfExtents() * 2.0;
    int axis = 0;
    if (ext.y() > ext.x()) axis = 1;
    if (ext.z() > ext[axis]) axis = 2;
    float splitPos = box.min[axis] + ext[axis]*0.5f;

    // partition
    std::vector<size_t> left, right;
    left.reserve(prims.size()); right.reserve(prims.size());
    for (auto idx : prims) {
        glm::vec3 c = (vertices[tris[idx].x] +
                       vertices[tris[idx].y] +
                       vertices[tris[idx].z]) / 3.0f;
        if (c[axis] < splitPos) left.push_back(idx);
        else                    right.push_back(idx);
    }
    if (left.empty() || right.empty()) {
        size_t mid = prims.size()/2;
        left  = std::vector<size_t>(prims.begin(), prims.begin()+mid);
        right = std::vector<size_t>(prims.begin()+mid, prims.end());
    }
    node->left  = buildRecursive(left,  vertices, tris, depth+1);
    node->right = buildRecursive(right, vertices, tris, depth+1);
    return node;
}

AABB BVH::computeTriangleBounds(
    const glm::vec3& v0,
    const glm::vec3& v1,
    const glm::vec3& v2) const {
    glm::vec3 mn = glm::min(glm::min(v0, v1), v2);
    glm::vec3 mx = glm::max(glm::max(v0, v1), v2);
    return AABB(
        Eigen::Vector3d(mn.x, mn.y, mn.z),
        Eigen::Vector3d(mx.x, mx.y, mx.z)
    );
}