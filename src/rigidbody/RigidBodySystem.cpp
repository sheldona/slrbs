#include "rigidbody/RigidBodySystem.h"
#include "integrator/IntegrationMethod.h"

#include "collision/CollisionDetect.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "solvers/SolverBoxPGS.h"
#include "solvers/SolverBoxBPP.h"
#include "solvers/SolverConjGradient.h"
#include "solvers/SolverConjResidual.h"
#include "solvers/SolverPGSSM.h"
#include "solvers/SolverProximal.h"
#include "integrator/Integrator.h"

#ifdef USE_OPENMP
#   include <omp.h>
#endif

namespace {
    static Solver* s_solvers[6] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

    // Graph coloring algorithm for rigid bodies
    void colorRigidBodies(std::vector<RigidBody*>& bodies,
                          const std::vector<Joint*>& joints,
                          const std::vector<Contact*>& contacts) {
        // Reset all colors
        for (auto b : bodies) {
            b->color = -1; // Uncolored
        }

        // Create adjacency list representation of the constraint graph
        std::vector<std::vector<int>> adjacency(bodies.size());

        // Add edges for joints
        for (const auto& joint : joints) {
            if (!joint->body0 || !joint->body1) continue;

            int idx0 = -1, idx1 = -1;
            for (size_t i = 0; i < bodies.size(); ++i) {
                if (bodies[i] == joint->body0) idx0 = static_cast<int>(i);
                if (bodies[i] == joint->body1) idx1 = static_cast<int>(i);
            }

            if (idx0 >= 0 && idx1 >= 0) {
                adjacency[idx0].push_back(idx1);
                adjacency[idx1].push_back(idx0);
            }
        }

        // Add edges for contacts
        for (const auto& contact : contacts) {
            if (!contact->body0 || !contact->body1) continue;

            int idx0 = -1, idx1 = -1;
            for (size_t i = 0; i < bodies.size(); ++i) {
                if (bodies[i] == contact->body0) idx0 = static_cast<int>(i);
                if (bodies[i] == contact->body1) idx1 = static_cast<int>(i);
            }

            if (idx0 >= 0 && idx1 >= 0) {
                adjacency[idx0].push_back(idx1);
                adjacency[idx1].push_back(idx0);
            }
        }

        // Greedy graph coloring algorithm
        std::vector<bool> available(bodies.size(), true);
        int maxColor = 0;

        for (size_t i = 0; i < bodies.size(); ++i) {
            // Mark colors of adjacent vertices as unavailable
            for (int neighbor : adjacency[i]) {
                if (bodies[neighbor]->color >= 0) {
                    available[bodies[neighbor]->color] = false;
                }
            }

            // Find the first available color
            int color;
            for (color = 0; color < static_cast<int>(bodies.size()); ++color) {
                if (available[color]) break;
            }

            // Assign color to this body
            bodies[i]->color = color;
            maxColor = std::max(maxColor, color);

            // Reset available colors for next iteration
            for (int neighbor : adjacency[i]) {
                if (bodies[neighbor]->color >= 0) {
                    available[bodies[neighbor]->color] = true;
                }
            }
        }

        // Store the number of colors for reference
        for (auto b : bodies) {
            b->numColors = maxColor + 1;
        }
    }
}

RigidBodySystem::RigidBodySystem()
 : m_collisionsEnabled(true)
 , m_gravity(0.0f, -9.81f, 0.0f)
 , m_solverType(SolverType::PGS)
 , m_solverIter(10)
 , m_integrationMethod(IntegrationMethod::EXPLICIT_EULER)
 , m_useGraphColoring(true)
 , m_useOpenMP(true)
 , m_useSolverOpenMP(true)
 , m_useCollisionOpenMP(true)
 , m_boxPGSStabilizationFactor(0.3f)
 , m_pgssmSubIterations(3)
 , m_pgssmGamma(0.3f)
{
    m_collisionDetect = std::make_unique<CollisionDetect>(this);

    // Instantiate one solver of each type (lazy alternative: factory on demand)
    s_solvers[0] = new SolverBoxPGS(this);
    s_solvers[1] = new SolverConjGradient(this);
    s_solvers[2] = new SolverConjResidual(this);
    s_solvers[3] = new SolverPGSSM(this);
    s_solvers[4] = new SolverProximal(this);
    s_solvers[5] = new SolverBoxBPP(this);

    // Initialize solver parameters
    setBoxPGSStabilizationFactor(m_boxPGSStabilizationFactor);
    setPGSSMSubIterations(m_pgssmSubIterations);
    setPGSSMGamma(m_pgssmGamma);
}

RigidBodySystem::~RigidBodySystem() {
    clear();
    for (auto& s : s_solvers) { delete s; s = nullptr; }
}

void RigidBodySystem::addBody(RigidBody* b)  { m_bodies.push_back(b); }
void RigidBodySystem::addJoint(Joint*  j)    {
    m_joints.push_back(j);
    if (j->body0) j->body0->joints.push_back(j);
    if (j->body1) j->body1->joints.push_back(j);
}

void RigidBodySystem::step(float dt)
{
#ifdef USE_OPENMP
    #pragma omp parallel for if(m_useOpenMP && m_bodies.size() > 16)
#endif
    for (size_t i = 0; i < m_bodies.size(); ++i) {
        auto b = m_bodies[i];
        b->f    = b->mass * m_gravity;
        b->tau.setZero();
        b->fc.setZero();
        b->tauc.setZero();
        b->contacts.clear();
        b->gsDamp.setZero();
    }

    computeInertias();
    if (m_preStepFunc) m_preStepFunc(*this, dt);

    m_collisionDetect->clear();
    if (m_collisionsEnabled) {
        // Pass OpenMP flag to collision detection system
        m_collisionDetect->setUseOpenMP(m_useCollisionOpenMP);

        m_collisionDetect->detectCollisions();
        m_collisionDetect->computeContactJacobians();

        // — geometric-stiffness damping pass —
        if (m_enableGSDamping) {
            // zero out per-body accumulators
            for (auto b : m_bodies) {
                b->gsSum.setZero();
            }
            // accumulate from joints
            for (auto j : m_joints) {
                j->computeGeometricStiffness();
                j->body0->gsSum += j->G0;
                j->body1->gsSum += j->G1;
            }
            // accumulate from contacts
            auto& ctrs = m_collisionDetect->getContacts();
            for (auto c : ctrs) {
                c->computeGeometricStiffness();
                c->body0->gsSum += c->G0;
                c->body1->gsSum += c->G1;
            }
            // convert to damping and update each body's inertia
            for (auto b : m_bodies) {
                if (b->fixed) continue;
                Eigen::Vector3f alphaDamp;
                for (int k = 0; k < 3; ++k) {
                    float stiffness = b->gsSum.col(k+3).norm();
                    alphaDamp[k] = m_gsAlpha * stiffness;
                }
                b->gsDamp = alphaDamp;
                b->updateInertiaMatrix();
            }
        }
    }

    // Apply graph coloring if enabled
    if (m_useGraphColoring) {
        colorRigidBodies(m_bodies, m_joints, m_collisionDetect->getContacts());
    }

#ifdef USE_OPENMP
    // Compute joint Jacobians, using coloring if enabled
    if (m_useGraphColoring && !m_joints.empty()) {
        int numColors = m_bodies.empty() ? 0 : m_bodies[0]->numColors;
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(m_useOpenMP)
            for (size_t i = 0; i < m_joints.size(); ++i) {
                Joint* j = m_joints[i];
                bool process = (j->body0 && j->body0->color == color)
                            || (j->body1 && j->body1->color == color);
                if (process) {
                    j->computeJacobian();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_useOpenMP && m_joints.size() > 16)
        for (auto& j : m_joints) {
            j->computeJacobian();
        }
    }
#else
    for (auto& j : m_joints) {
        j->computeJacobian();
    }
#endif

#ifdef USE_OPENMP
    #pragma omp parallel for if(m_useOpenMP && m_bodies.size() > 16)
#endif
    for (size_t i = 0; i < m_bodies.size(); ++i) {
        auto b = m_bodies[i];
        b->fc.setZero();
        b->tauc.setZero();
    }

    calcConstraintForces(dt);

    // Create and use an integrator based on the selected method
    auto integrator = createIntegrator(m_integrationMethod, m_useOpenMP);
    integrator->integrate(*this, dt);
}

void RigidBodySystem::clear() {
    if (m_resetFunc) m_resetFunc();
    m_collisionDetect->clear();
    for (auto j : m_joints) delete j;
    m_joints.clear();
    for (auto b : m_bodies) delete b;
    m_bodies.clear();
}

void RigidBodySystem::computeInertias() {
#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(m_useOpenMP)
            for (size_t i = 0; i < m_bodies.size(); ++i) {
                if (m_bodies[i]->color == color) {
                    m_bodies[i]->updateInertiaMatrix();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_useOpenMP && m_bodies.size() > 16)
        for (size_t i = 0; i < m_bodies.size(); ++i)
            m_bodies[i]->updateInertiaMatrix();
    }
#else
    for (size_t i = 0; i < m_bodies.size(); ++i)
        m_bodies[i]->updateInertiaMatrix();
#endif
}

const std::vector<Contact*>& RigidBodySystem::getContacts() const {
    return m_collisionDetect->getContacts();
}
std::vector<Contact*>& RigidBodySystem::getContacts() {
    return m_collisionDetect->getContacts();
}

void RigidBodySystem::calcConstraintForces(float dt) {
    int idx = static_cast<int>(m_solverType);
    s_solvers[idx]->setMaxIter(m_solverIter);

    // Pass OpenMP flag to the solver
    s_solvers[idx]->setUseOpenMP(m_useSolverOpenMP);

    s_solvers[idx]->solve(dt);

#ifdef USE_OPENMP
    if (m_useGraphColoring && !m_joints.empty() && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(m_useSolverOpenMP)
            for (size_t i = 0; i < m_joints.size(); ++i) {
                Joint* j = m_joints[i];
                bool processBody0 = j->body0 && j->body0->color == color;
                bool processBody1 = j->body1 && j->body1->color == color;

                Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
                Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;

                if (processBody0) {
                    j->body0->fc   += f0.head<3>();
                    j->body0->tauc += f0.tail<3>();
                }

                if (processBody1) {
                    j->body1->fc   += f1.head<3>();
                    j->body1->tauc += f1.tail<3>();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_useSolverOpenMP && m_joints.size() > 16)
        for (auto j : m_joints) {
            Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
            Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;
            #pragma omp critical
            {
                j->body0->fc   += f0.head<3>();
                j->body0->tauc += f0.tail<3>();
                j->body1->fc   += f1.head<3>();
                j->body1->tauc += f1.tail<3>();
            }
        }
    }
#else
    for (auto j : m_joints) {
        Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
        Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;
        j->body0->fc   += f0.head<3>();
        j->body0->tauc += f0.tail<3>();
        j->body1->fc   += f1.head<3>();
        j->body1->tauc += f1.tail<3>();
    }
#endif

    auto contacts = m_collisionDetect->getContacts();
#ifdef USE_OPENMP
    if (m_useGraphColoring && !contacts.empty() && !m_bodies.empty()) {
        int numColors = m_bodies[0]->numColors;

        // Process each color group in sequence
        for (int color = 0; color < numColors; ++color) {
            #pragma omp parallel for if(m_useSolverOpenMP)
            for (size_t i = 0; i < contacts.size(); ++i) {
                Contact* c = contacts[i];
                bool processBody0 = c->body0 && c->body0->color == color && !c->body0->fixed;
                bool processBody1 = c->body1 && c->body1->color == color && !c->body1->fixed;

                Eigen::Vector6f f0 = c->J0.transpose() * c->lambda / dt;
                Eigen::Vector6f f1 = c->J1.transpose() * c->lambda / dt;

                if (processBody0) {
                    c->body0->fc   += f0.head<3>();
                    c->body0->tauc += f0.tail<3>();
                }

                if (processBody1) {
                    c->body1->fc   += f1.head<3>();
                    c->body1->tauc += f1.tail<3>();
                }
            }
        }
    } else {
        #pragma omp parallel for if(m_useSolverOpenMP && contacts.size() > 16)
        for (auto c : contacts) {
            Eigen::Vector6f f0 = c->J0.transpose() * c->lambda / dt;
            Eigen::Vector6f f1 = c->J1.transpose() * c->lambda / dt;
            if (!c->body0->fixed) {
                #pragma omp critical
                {
                    c->body0->fc   += f0.head<3>();
                    c->body0->tauc += f0.tail<3>();
                }
            }
            if (!c->body1->fixed) {
                #pragma omp critical
                {
                    c->body1->fc   += f1.head<3>();
                    c->body1->tauc += f1.tail<3>();
                }
            }
        }
    }
#else
    for (auto c : contacts) {
        Eigen::Vector6f f0 = c->J0.transpose() * c->lambda / dt;
        Eigen::Vector6f f1 = c->J1.transpose() * c->lambda / dt;
        if (!c->body0->fixed) {
            c->body0->fc   += f0.head<3>();
            c->body0->tauc += f0.tail<3>();
        }
        if (!c->body1->fixed) {
            c->body1->fc   += f1.head<3>();
            c->body1->tauc += f1.tail<3>();
        }
    }
#endif
}
// Newton integrator parameters
void RigidBodySystem::setNewtonMaxIterations(int iters) {
    if (iters < 1) {
        m_maxIterations = 1;
    } else {
        m_maxIterations = iters;
    }
}

void RigidBodySystem::setNewtonTolerance(float tol) {
    if (tol <= 0.0f) {
        m_tolerance = 1e-6f;
    } else {
        m_tolerance = tol;
    }
}

void RigidBodySystem::setNewtonDamping(float damping) {
    if (damping < 0.0f || damping > 1.0f) {
        m_damping = std::clamp(damping, 0.0f, 1.0f);
    } else {
        m_damping = damping;
    }
}

// For Proximal solver
void RigidBodySystem::setProximalAbsTolerance(float tol) {
    if (m_solverType != SolverType::PROXIMAL) {
        return;
    }

    SolverProximal* proximal = dynamic_cast<SolverProximal*>(s_solvers[static_cast<int>(SolverType::PROXIMAL)]);
    if (proximal) {
        proximal->setAbsoluteTolerance(tol);
    }
}

void RigidBodySystem::setProximalRelTolerance(float tol) {
    if (m_solverType != SolverType::PROXIMAL) {
        return;
    }

    SolverProximal* proximal = dynamic_cast<SolverProximal*>(s_solvers[static_cast<int>(SolverType::PROXIMAL)]);
    if (proximal) {
        proximal->setRelativeTolerance(tol);
    }
}

Solver* RigidBodySystem::getProximalSolver() {
    return s_solvers[static_cast<int>(SolverType::PROXIMAL)];
}

// For conjugate gradient solvers
void RigidBodySystem::setConjTolerance(float tol) {
    if (tol <= 0.0f) {
        tol = 1e-6f;
    }

    if (m_solverType == SolverType::CONJ_GRADIENT) {
        SolverConjGradient* cg = dynamic_cast<SolverConjGradient*>(s_solvers[static_cast<int>(SolverType::CONJ_GRADIENT)]);
        if (cg) {
            cg->setTolerance(tol);
        }
    } else if (m_solverType == SolverType::CONJ_RESIDUAL) {
        SolverConjResidual* cr = dynamic_cast<SolverConjResidual*>(s_solvers[static_cast<int>(SolverType::CONJ_RESIDUAL)]);
        if (cr) {
            cr->setTolerance(tol);
        }
    }
}

float RigidBodySystem::getConjTolerance() const {
    if (m_solverType == SolverType::CONJ_GRADIENT) {
        const SolverConjGradient* cg = dynamic_cast<const SolverConjGradient*>(s_solvers[static_cast<int>(SolverType::CONJ_GRADIENT)]);
        if (cg) {
            return cg->getTolerance();
        }
    } else if (m_solverType == SolverType::CONJ_RESIDUAL) {
        const SolverConjResidual* cr = dynamic_cast<const SolverConjResidual*>(s_solvers[static_cast<int>(SolverType::CONJ_RESIDUAL)]);
        if (cr) {
            return cr->getTolerance();
        }
    }
    return m_tolerance; // Default to system tolerance if solver-specific not available
}

void RigidBodySystem::setConjRestartInterval(int interval) {
    if (interval < 1) {
        interval = 1;
    }

    if (m_solverType == SolverType::CONJ_GRADIENT) {
        SolverConjGradient* cg = dynamic_cast<SolverConjGradient*>(s_solvers[static_cast<int>(SolverType::CONJ_GRADIENT)]);
        if (cg) {
            cg->setRestartInterval(interval);
        }
    } else if (m_solverType == SolverType::CONJ_RESIDUAL) {
        SolverConjResidual* cr = dynamic_cast<SolverConjResidual*>(s_solvers[static_cast<int>(SolverType::CONJ_RESIDUAL)]);
        if (cr) {
            cr->setRestartInterval(interval);
        }
    }
}

int RigidBodySystem::getConjRestartInterval() const {
    if (m_solverType == SolverType::CONJ_GRADIENT) {
        const SolverConjGradient* cg = dynamic_cast<const SolverConjGradient*>(s_solvers[static_cast<int>(SolverType::CONJ_GRADIENT)]);
        if (cg) {
            return cg->getRestartInterval();
        }
    } else if (m_solverType == SolverType::CONJ_RESIDUAL) {
        const SolverConjResidual* cr = dynamic_cast<const SolverConjResidual*>(s_solvers[static_cast<int>(SolverType::CONJ_RESIDUAL)]);
        if (cr) {
            return cr->getRestartInterval();
        }
    }
    return 10; // Default restart interval if not set
}

// Generic parameter setters
void RigidBodySystem::setMaxIterations(int iterations) {
    if (iterations < 1) {
        m_maxIterations = 1;
    } else {
        m_maxIterations = iterations;
    }
}

void RigidBodySystem::setTolerance(float tolerance) {
    if (tolerance <= 0.0f) {
        m_tolerance = 1e-6f;
    } else {
        m_tolerance = tolerance;
    }
}

void RigidBodySystem::setDamping(float damping) {
    if (damping < 0.0f || damping > 1.0f) {
        m_damping = std::clamp(damping, 0.0f, 1.0f);
    } else {
        m_damping = damping;
    }
}

// BoxBPP solver parameters
void RigidBodySystem::setBoxBPPMaxIterations(int iters) {
    if (iters < 1) {
        m_boxBPPMaxIterations = 1;
    } else {
        m_boxBPPMaxIterations = iters;
    }

    // If BPP is the current solver, update it directly
    if (m_solverType == SolverType::BPP) {
        SolverBoxBPP* bpp = dynamic_cast<SolverBoxBPP*>(s_solvers[static_cast<int>(SolverType::BPP)]);
        if (bpp) {
            bpp->setMaxIter(m_boxBPPMaxIterations);
        }
    }
}

int RigidBodySystem::getBoxBPPMaxIterations() const {
    if (m_solverType == SolverType::BPP) {
        SolverBoxBPP* bpp = dynamic_cast<SolverBoxBPP*>(s_solvers[static_cast<int>(SolverType::BPP)]);
        if (bpp) {
            return bpp->getMaxIter();
        }
    }
    return m_boxBPPMaxIterations;
}

void RigidBodySystem::setBoxBPPStabilization(float stabilization) {
    if (stabilization <= 0.0f) {
        m_boxBPPStabilization = 1.0f;
    } else {
        m_boxBPPStabilization = stabilization;
    }

    // If BPP is the current solver, update it directly
    if (m_solverType == SolverType::BPP) {
        SolverBoxBPP* bpp = dynamic_cast<SolverBoxBPP*>(s_solvers[static_cast<int>(SolverType::BPP)]);
        if (bpp) {
            bpp->setStabilization(m_boxBPPStabilization);
        }
    }
}

float RigidBodySystem::getBoxBPPStabilization() const {
    return m_boxBPPStabilization;
}

void RigidBodySystem::setBoxBPPPivotTolerance(float tol) {
    if (tol <= 0.0f) {
        m_boxBPPPivotTolerance = 1e-5f;
    } else {
        m_boxBPPPivotTolerance = tol;
    }

    // If BPP is the current solver, update it directly
    if (m_solverType == SolverType::BPP) {
        SolverBoxBPP* bpp = dynamic_cast<SolverBoxBPP*>(s_solvers[static_cast<int>(SolverType::BPP)]);
        if (bpp) {
            bpp->setPivotTolerance(m_boxBPPPivotTolerance);
        }
    }
}

float RigidBodySystem::getBoxBPPPivotTolerance() const {
    return m_boxBPPPivotTolerance;
}

// BoxPGS Solver parameters
void RigidBodySystem::setBoxPGSStabilizationFactor(float factor) {
    if (factor < 0.0f || factor > 1.0f) {
        m_boxPGSStabilizationFactor = std::clamp(factor, 0.0f, 1.0f);
    } else {
        m_boxPGSStabilizationFactor = factor;
    }

    SolverBoxPGS* pgs = dynamic_cast<SolverBoxPGS*>(s_solvers[static_cast<int>(SolverType::PGS)]);
    if (pgs) {
        pgs->setStabilizationFactor(m_boxPGSStabilizationFactor);
    }
}

float RigidBodySystem::getBoxPGSStabilizationFactor() const {
    if (m_solverType == SolverType::PGS) {
        const SolverBoxPGS* pgs = dynamic_cast<const SolverBoxPGS*>(s_solvers[static_cast<int>(SolverType::PGS)]);
        if (pgs) {
            return pgs->getStabilizationFactor();
        }
    }
    return m_boxPGSStabilizationFactor;
}

// PGSSM Solver parameters
void RigidBodySystem::setPGSSMSubIterations(int subIter) {
    if (subIter < 1) {
        m_pgssmSubIterations = 1;
    } else {
        m_pgssmSubIterations = subIter;
    }

    SolverPGSSM* pgssm = dynamic_cast<SolverPGSSM*>(s_solvers[static_cast<int>(SolverType::PGSSM)]);
    if (pgssm) {
        pgssm->setSubIterations(m_pgssmSubIterations);
    }
}

int RigidBodySystem::getPGSSMSubIterations() const {
    if (m_solverType == SolverType::PGSSM) {
        const SolverPGSSM* pgssm = dynamic_cast<const SolverPGSSM*>(s_solvers[static_cast<int>(SolverType::PGSSM)]);
        if (pgssm) {
            return pgssm->getSubIterations();
        }
    }
    return m_pgssmSubIterations;
}

void RigidBodySystem::setPGSSMGamma(float gamma) {
    if (gamma < 0.0f || gamma > 1.0f) {
        m_pgssmGamma = std::clamp(gamma, 0.0f, 1.0f);
    } else {
        m_pgssmGamma = gamma;
    }

    SolverPGSSM* pgssm = dynamic_cast<SolverPGSSM*>(s_solvers[static_cast<int>(SolverType::PGSSM)]);
    if (pgssm) {
        pgssm->setGamma(m_pgssmGamma);
    }
}

float RigidBodySystem::getPGSSMGamma() const {
    if (m_solverType == SolverType::PGSSM) {
        const SolverPGSSM* pgssm = dynamic_cast<const SolverPGSSM*>(s_solvers[static_cast<int>(SolverType::PGSSM)]);
        if (pgssm) {
            return pgssm->getGamma();
        }
    }
    return m_pgssmGamma;
}