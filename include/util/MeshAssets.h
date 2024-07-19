#pragma once

#include <Eigen/Dense>
#include <string>
#include <map>

// Data structure for a polygon mesh.
struct Mesh
{
    Mesh() : name(""), meshV(), meshF() { }

    std::string name;
    Eigen::MatrixXf meshV;
    Eigen::MatrixXi meshF;
};


// Cache of mesh data. Each filename is loaded only once.
typedef std::map<unsigned int, Mesh> MeshCache;

// The mesh registry is a static class storing geometry and material information
// for drawing rigid bodies.
//
class MeshAssetRegistry
{
public:

    // Loads a mesh from the OBJ file @a _filename
    // and adds it to the cache.  Returns a pointer to the mesh in the cache.
    static Mesh* loadObj(const std::string& _filename);

    // Clears the mesh cache.
    static void clear();

    // Returns the container of cached meshes.
    static MeshCache& cachedMeshes();

private:

    // The one and only instance of the mesh cache.
    static MeshCache m_meshCache;
};

Mesh createCylinder(size_t N, float radius, float height);
Mesh createPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& n);
Mesh createBox(const Eigen::Vector3f& dim);
Mesh createSphere(float radius);
