#include "util/MeshAssets.h"
#include "util/OBJLoader.h"
#include <cassert>
#include <functional>

#define _USE_MATH_DEFINES
#include <math.h>

namespace
{

    struct Edge
    {
        uint32_t v0;
        uint32_t v1;

        Edge(uint32_t v0, uint32_t v1)
            : v0(v0 < v1 ? v0 : v1)
            , v1(v0 < v1 ? v1 : v0)
        {
        }

        bool operator <(const Edge& rhs) const
        {
            return (v0 < rhs.v0) || (v0 == rhs.v0 && v1 < rhs.v1);
        }
    };

    uint32_t subdivideEdge(uint32_t f0, uint32_t f1, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, std::vector<Eigen::Vector3f>& verts, std::map<Edge, uint32_t>& io_divisions)
    {
        const Edge edge(f0, f1);
        auto it = io_divisions.find(edge);
        if (it != io_divisions.end())
        {
            return it->second;
        }

        const Eigen::Vector3f v = (0.5f * (v0 + v1)).normalized();
        const uint32_t f = verts.size();
        verts.push_back(v);
        io_divisions.emplace(edge, f);
        return f;
    }

    void subdivideMesh(const Mesh& meshIn, Mesh& meshOut)
    {
        const unsigned int nVerts = meshIn.meshV.rows();
        std::vector<Eigen::Vector3f> verts(nVerts);
        for (unsigned int i = 0; i < nVerts; ++i)
        {
            verts[i] = meshIn.meshV.row(i);
        }

        const unsigned int nFaces = meshIn.meshF.rows();
        std::vector<Eigen::Vector3i> faces(nFaces);
        for (unsigned int i = 0; i < nFaces; ++i)
        {
            faces[i] = meshIn.meshF.row(i);
        }

        std::map<Edge, uint32_t> divisions;

        for (unsigned int i = 0; i < nFaces; ++i)
        {
            const uint32_t f0 = faces[i][0];
            const uint32_t f1 = faces[i][1];
            const uint32_t f2 = faces[i][2];

            const Eigen::Vector3f v0 = verts[f0];
            const Eigen::Vector3f v1 = verts[f1];
            const Eigen::Vector3f v2 = verts[f2];

            const uint32_t f3 = subdivideEdge(f0, f1, v0, v1, verts, divisions);
            const uint32_t f4 = subdivideEdge(f1, f2, v1, v2, verts, divisions);
            const uint32_t f5 = subdivideEdge(f2, f0, v2, v0, verts, divisions);

            faces.push_back(Eigen::Vector3i(f0, f3, f5));
            faces.push_back(Eigen::Vector3i(f3, f1, f4));
            faces.push_back(Eigen::Vector3i(f4, f2, f5));
            faces.push_back(Eigen::Vector3i(f3, f4, f5));
        }

        meshOut.meshV.resize(verts.size(), 3);
        for (unsigned int i = 0; i < verts.size(); ++i)
        {
            meshOut.meshV.row(i) = verts[i];
        }

        meshOut.meshF.resize(faces.size(), 3);
        for (unsigned int i = 0; i < faces.size(); ++i)
        {
            meshOut.meshF.row(i) = faces[i];
        }
    }

}

std::map<unsigned int, Mesh> MeshAssetRegistry::m_meshCache = std::map<unsigned int, Mesh>();

Mesh* MeshAssetRegistry::loadObj(const std::string& _filename)
{
    const unsigned int key = std::hash<std::string>{}(_filename);
    auto cacheItr = m_meshCache.find(key);

    if (cacheItr != m_meshCache.end())
    {
        return &(cacheItr->second);
    }

    Mesh mesh;
    OBJLoader::load(_filename, mesh.meshV, mesh.meshF);

    m_meshCache[key] = mesh;
    return &(m_meshCache[key]);
}

void MeshAssetRegistry::clear()
{
    m_meshCache.clear();
}

MeshCache& MeshAssetRegistry::cachedMeshes()
{
    return m_meshCache;
}



Mesh createCylinder(size_t N, float radius, float height)
{
    Mesh mesh;

    // generate vertices
    mesh.meshV.setZero(2 * N + 2, 3);
    for (size_t i = 0; i < N; i++) {
        float ratio = static_cast<float>(i) / N;
        float ang = ratio * (M_PI * 2.0f);
        float x = std::cos(ang) * radius;
        float z = std::sin(ang) * radius;
        // bottom vertex
        mesh.meshV.row(2 * i) = Eigen::Vector3f(x, -0.5f * height, z);
        // top vertex
        mesh.meshV.row(2 * i + 1) = Eigen::Vector3f(x, 0.5f * height, z);
    }
    // end caps
    mesh.meshV.row(2 * N) = Eigen::Vector3f(0.0f, -0.5f * height, 0.0f);
    mesh.meshV.row(2 * N + 1) = Eigen::Vector3f(0.0f, 0.5f * height, 0.0f);

    mesh.meshF.setZero(2 * N + 2 * N, 3);
    // add faces around the cylinder
    for (size_t i = 0; i < N; i++) {
        const int ii = i * 2; // bottom
        const int jj = (ii + 1) % (N * 2);  // top
        const int kk = (ii + 2) % (N * 2);  // bottom
        const int ll = (ii + 3) % (N * 2);  // top
        mesh.meshF.row(2 * i) = Eigen::Vector3i(ii, kk, ll);
        mesh.meshF.row(2 * i + 1) = Eigen::Vector3i(ll, jj, ii);
    }

    // add faces at bottom cap
    for (size_t i = 0; i < N; i++) {
        const int ii = 2 * N;
        const int jj = i * 2;
        const int kk = (jj + 2) % (N * 2);
        mesh.meshF.row(2 * N + i) = Eigen::Vector3i(ii, kk, jj);
    }
    // add faces at top cap
    for (size_t i = 0; i < N; i++) {
        const int ii = 2 * N + 1;
        const int jj = i * 2 + 1; // top
        const int kk = (jj + 2) % (N * 2);  // top
        mesh.meshF.row(3 * N + i) = Eigen::Vector3i(ii, kk, jj);
    }

    return mesh;
}

Mesh createBox(const Eigen::Vector3f& dim)
{
    Mesh mesh;

    const float x = 0.5f * dim(0);
    const float y = 0.5f * dim(1);
    const float z = 0.5f * dim(2);

    // generate vertices
    mesh.meshV.setZero(8, 3);
    mesh.meshV.row(0) = Eigen::Vector3f(x, y, z);
    mesh.meshV.row(1) = Eigen::Vector3f(-x, y, z);
    mesh.meshV.row(2) = Eigen::Vector3f(-x, -y, z);
    mesh.meshV.row(3) = Eigen::Vector3f(x, -y, z);
    mesh.meshV.row(4) = Eigen::Vector3f(x, y, -z);
    mesh.meshV.row(5) = Eigen::Vector3f(-x, y, -z);
    mesh.meshV.row(6) = Eigen::Vector3f(-x, -y, -z);
    mesh.meshV.row(7) = Eigen::Vector3f(x, -y, -z);

    // generate faces
    mesh.meshF.setZero(12, 3);
    mesh.meshF.row(0) = Eigen::Vector3i(0, 1, 2);
    mesh.meshF.row(1) = Eigen::Vector3i(2, 3, 0);
    mesh.meshF.row(2) = Eigen::Vector3i(0, 3, 7);
    mesh.meshF.row(3) = Eigen::Vector3i(7, 4, 0);
    mesh.meshF.row(4) = Eigen::Vector3i(1, 5, 6);
    mesh.meshF.row(5) = Eigen::Vector3i(6, 2, 1);
    mesh.meshF.row(6) = Eigen::Vector3i(5, 4, 7);
    mesh.meshF.row(7) = Eigen::Vector3i(7, 6, 5);
    mesh.meshF.row(8) = Eigen::Vector3i(0, 4, 5);
    mesh.meshF.row(9) = Eigen::Vector3i(5, 1, 0);
    mesh.meshF.row(10) = Eigen::Vector3i(7, 3, 6);
    mesh.meshF.row(11) = Eigen::Vector3i(6, 3, 2);

    return mesh;
}

Mesh createSphere(float radius)
{
    const float t = 0.5f * (1.0 + std::sqrtf(5.0f));

    Mesh mesh;

    mesh.meshV.setZero(12, 3);
    // Vertices
    mesh.meshV.row(0) = Eigen::Vector3f(-1.0, t, 0.0).normalized();
    mesh.meshV.row(1) = Eigen::Vector3f(1.0, t, 0.0).normalized();
    mesh.meshV.row(2) = Eigen::Vector3f(-1.0, -t, 0.0).normalized();
    mesh.meshV.row(3) = Eigen::Vector3f(1.0, -t, 0.0).normalized();
    mesh.meshV.row(4) = Eigen::Vector3f(0.0, -1.0, t).normalized();
    mesh.meshV.row(5) = Eigen::Vector3f(0.0, 1.0, t).normalized();
    mesh.meshV.row(6) = Eigen::Vector3f(0.0, -1.0, -t).normalized();
    mesh.meshV.row(7) = Eigen::Vector3f(0.0, 1.0, -t).normalized();
    mesh.meshV.row(8) = Eigen::Vector3f(t, 0.0, -1.0).normalized();
    mesh.meshV.row(9) = Eigen::Vector3f(t, 0.0, 1.0).normalized();
    mesh.meshV.row(10) = Eigen::Vector3f(-t, 0.0, -1.0).normalized();
    mesh.meshV.row(11) = Eigen::Vector3f(-t, 0.0, 1.0).normalized();

    // Faces
    mesh.meshF.setZero(20, 3);
    mesh.meshF.row(0) = Eigen::Vector3i({ 0, 11, 5 });
    mesh.meshF.row(1) = Eigen::Vector3i({ 0, 5, 1 });
    mesh.meshF.row(2) = Eigen::Vector3i({ 0, 1, 7 });
    mesh.meshF.row(3) = Eigen::Vector3i({ 0, 7, 10 });
    mesh.meshF.row(4) = Eigen::Vector3i({ 0, 10, 11 });
    mesh.meshF.row(5) = Eigen::Vector3i({ 1, 5, 9 });
    mesh.meshF.row(6) = Eigen::Vector3i({ 5, 11, 4 });
    mesh.meshF.row(7) = Eigen::Vector3i({ 11, 10, 2 });
    mesh.meshF.row(8) = Eigen::Vector3i({ 10, 7, 6 });
    mesh.meshF.row(9) = Eigen::Vector3i({ 7, 1, 8 });
    mesh.meshF.row(10) = Eigen::Vector3i({ 3, 9, 4 });
    mesh.meshF.row(11) = Eigen::Vector3i({ 3, 4, 2 });
    mesh.meshF.row(12) = Eigen::Vector3i({ 3, 2, 6 });
    mesh.meshF.row(13) = Eigen::Vector3i({ 3, 6, 8 });
    mesh.meshF.row(14) = Eigen::Vector3i({ 3, 8, 9 });
    mesh.meshF.row(15) = Eigen::Vector3i({ 4, 9, 5 });
    mesh.meshF.row(16) = Eigen::Vector3i({ 2, 4, 11 });
    mesh.meshF.row(17) = Eigen::Vector3i({ 6, 2, 10 });
    mesh.meshF.row(18) = Eigen::Vector3i({ 8, 6, 7 });
    mesh.meshF.row(19) = Eigen::Vector3i({ 9, 8, 1 });

    // Refine the mesh by subdividing twice
    Mesh tmp;
    subdivideMesh(mesh, tmp);
    subdivideMesh(tmp, mesh);

    mesh.meshV *= radius;

    return mesh;
}

Mesh createPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& n)
{
    static const Eigen::Vector3f x = Eigen::Vector3f(1, 0, 0);
    static const Eigen::Vector3f z = Eigen::Vector3f(0, 0, 1);
    Eigen::Vector3f t, b;
    if (std::fabs(n.dot(x)) > 1e-3f)
    {
        b = n.cross(x);
    }
    else
    {
        b = n.cross(z);
    }
    b.normalize();
    t = b.cross(n);
    t.normalize();

    Mesh mesh;
    const float size = 10.0f;
    mesh.meshV.setZero(4, 3);
    mesh.meshV.row(0) = p - size * b - size * t;
    mesh.meshV.row(1) = p - size * b + size * t;
    mesh.meshV.row(2) = p + size * b - size * t;
    mesh.meshV.row(3) = p + size * b + size * t;

    mesh.meshF.setZero(2, 3);
    mesh.meshF.row(0) = Eigen::Vector3i({ 0, 2, 3 });
    mesh.meshF.row(1) = Eigen::Vector3i({ 3, 0, 1 });

    return mesh;
}
