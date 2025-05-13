#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include "AABB.h"
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

struct BVHNode {
    AABB bounds;  // Use our double-precision AABB
    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;
    std::vector<size_t> primitiveIndices;
};

class BVH {
public:
    explicit BVH(int maxDepth = 10);

    // Build from generic face types via wrapper
    template <typename FaceType>
    void build(const std::vector<glm::vec3>& vertices,
               const std::vector<FaceType>& triangles) {
        build(vertices, convertFacesToIVec3(triangles));
    }

    // Core build interface
    void build(const std::vector<glm::vec3>& vertices,
               const std::vector<glm::ivec3>& triangles);

    void setMaxDepth(int d) { m_maxDepth = d; }
    int getMaxDepth() const { return m_maxDepth; }
    const BVHNode* getRoot() const { return root.get(); }

private:
    std::unique_ptr<BVHNode> root;
    int m_maxDepth;

    // Overload for array<size_t,3> faces: convert then delegate
    std::unique_ptr<BVHNode> buildRecursive(
        const std::vector<size_t>& prims,
        const std::vector<glm::vec3>& vertices,
        const std::vector<std::array<size_t,3>>& triangles,
        int depth);

    // Core recursive on ivec3
    std::unique_ptr<BVHNode> buildRecursive(
        const std::vector<size_t>& prims,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::ivec3>& triangles,
        int depth);

    // Compute bounds of one triangle
    AABB computeTriangleBounds(const glm::vec3& v0,
                               const glm::vec3& v1,
                               const glm::vec3& v2) const;

    template <typename F>
    static std::vector<glm::ivec3> convertFacesToIVec3(
        const std::vector<F>& faces) {
        std::vector<glm::ivec3> out;
        out.reserve(faces.size());
        for (auto& f : faces) {
            out.emplace_back(f[0], f[1], f[2]);
        }
        return out;
    }
};