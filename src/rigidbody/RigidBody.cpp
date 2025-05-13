#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"
#include "joint/Joint.h"
#include "util/MeshAssets.h"
#include "polyscope/surface_mesh.h"

int RigidBody::counter = 0;

RigidBody::RigidBody(float _mass, Geometry* _geometry, const std::string& _filename)
 : fixed(false), mass(_mass), x(0,0,0), xdot(0,0,0), omega(0,0,0), q(1,0,0,0),
   Ibody(Eigen::Matrix3f::Identity()), IbodyInv(Eigen::Matrix3f::Identity()), Iinv(Eigen::Matrix3f::Zero()),
   f(0,0,0), tau(0,0,0), fc(0,0,0), tauc(0,0,0), geometry(_geometry), contacts(), joints(), mesh(nullptr)
{
    Ibody = geometry->computeInertia(mass);
    IbodyInv = Ibody.inverse();
    clearGeometricStiffness();

    if (!_filename.empty()) {
        auto* cachedMesh = MeshAssetRegistry::loadObj(_filename);
        if (cachedMesh) {
            mesh = polyscope::registerSurfaceMesh(std::to_string(counter), cachedMesh->meshV, cachedMesh->meshF);
            if (mesh) applyVisualProperties();
        }
    }
    contacts.clear();
    RigidBody::counter++;
}

RigidBody::RigidBody(float _mass, Geometry* _geometry, const Mesh& _mesh)
 : fixed(false), mass(_mass), x(0,0,0), xdot(0,0,0), omega(0,0,0), q(1,0,0,0),
   Ibody(Eigen::Matrix3f::Identity()), IbodyInv(Eigen::Matrix3f::Identity()), Iinv(Eigen::Matrix3f::Zero()),
   f(0,0,0), tau(0,0,0), fc(0,0,0), tauc(0,0,0), geometry(_geometry), contacts(), joints(), mesh(nullptr)
{
    Ibody = geometry->computeInertia(mass);
    IbodyInv = Ibody.inverse();
    clearGeometricStiffness();

    mesh = polyscope::registerSurfaceMesh(std::to_string(counter), _mesh.meshV, _mesh.meshF);
    if (mesh) applyVisualProperties();

    contacts.clear();
    RigidBody::counter++;
}

void RigidBody::updateInertiaMatrix() {
    if (!fixed) {
        I    = q * Ibody * q.inverse();
        Iinv = q * IbodyInv * q.inverse();
    } else {
        Iinv.setZero();
    }
}

void RigidBody::updateInertiaMatrixWithGs(float dt) {
    // First, update the base inertia
    updateInertiaMatrix();
    // Then fold in geometric-stiffness damping as extra diagonal
    float dt2 = dt*dt;
    // We only adjust the 3×3 rotational block (omega damping)
    Eigen::Matrix3f extra = dt2 * gsSum.topLeftCorner<3,3>();
    I += extra;
    Iinv = I.inverse();  // re-compute inverse
}

void RigidBody::clearGeometricStiffness() {
    gsSum.setZero();
    gsDamp.setZero();
}

void RigidBody::addGeometricStiffness(const GBlock& G) {
    gsSum += G;
}

const GBlock& RigidBody::getAccumulatedGs() const {
    return gsSum;
}

void RigidBody::addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force) {
    Eigen::Vector3f r = pos - x;
    f   += force;
    tau += r.cross(force);
}

void RigidBody::getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel) {
    Eigen::Vector3f r = pos - x;
    vel = xdot + r.cross(omega);
}

void RigidBody::applyVisualProperties() {
    if (!mesh) return;
    if (visualProperties.count("colorR")) {
        mesh->setSurfaceColor({ visualProperties.at("colorR"),
                                visualProperties.at("colorG"),
                                visualProperties.at("colorB") });
    }
    if (visualProperties.count("transparency")) mesh->setTransparency(visualProperties.at("transparency"));
    if (visualProperties.count("smoothShade")) mesh->setSmoothShade(visualProperties.at("smoothShade")>0.5f);
    if (visualProperties.count("edgeWidth"))  mesh->setEdgeWidth(visualProperties.at("edgeWidth"));
}