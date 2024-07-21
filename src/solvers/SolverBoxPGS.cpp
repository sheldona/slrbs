#include "solvers/SolverBoxPGS.h"

#include "contact/Contact.h"
#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>


namespace
{
    static const float stabilization = 20.0f;
    static const float alpha = stabilization * 2;
    static const float beta = stabilization * stabilization * 2;

    static inline void multAndSub(const JBlock& G, const Eigen::Vector3f& x, const Eigen::Vector3f& y, const float& a, Eigen::VectorXf& b)
    {
            b -= a * G.col(0) * x(0);
            b -= a * G.col(1) * x(1);
            b -= a * G.col(2) * x(2);
            b -= a * G.col(3) * y(0);
            b -= a * G.col(4) * y(1);
            b -= a * G.col(5) * y(2);
    }


	// Computes the right-hand side vector of the Schur complement system: 
    //      b = -phi/h - J*vel - dt*JMinv*force
    //
    static inline void buildRHS(Joint* j, float h, Eigen::VectorXf& b)
    {
        const float hinv = 1.0f / h;
        const int dim = j->lambda.rows();
        b = -hinv * j->phi * (h * beta / (h * beta + alpha));

        if( !j->body0->fixed )
        {
            multAndSub(j->J0Minv, j->body0->f, j->body0->tau, h, b);
            multAndSub(j->J0, j->body0->xdot, j->body0->omega, 1.0f, b);
        }
        if( !j->body1->fixed )
        {
            multAndSub(j->J1Minv, j->body1->f, j->body1->tau, h, b);
            multAndSub(j->J1, j->body1->xdot, j->body1->omega, 1.0f, b);
        }
    }

    // Loop over all other contacts for a body and compute modifications to the rhs vector b: 
    //           x -= (JMinv*Jother^T) * lambda_other
    //
    static inline void accumulateCoupledContactsAndJoints(Joint* j, const JBlock& JMinv, RigidBody* body, Eigen::VectorXf& b)
    {
        if( body->fixed )
            return;

        const int dim = j->lambda.rows();

        for(Contact* cc : body->contacts)
        {
            if( cc != j )
            {
                if( body == cc->body0 )
                    b -= JMinv * (cc->J0.transpose() * cc->lambda);
                else
                    b -= JMinv * (cc->J1.transpose() * cc->lambda);
            }
        }

        for (Joint* jj : body->joints)
        {
            if (jj != j)
            {
                const int otherDim = jj->lambda.rows();
                if (body == jj->body0)
                {
                    b -= JMinv * (jj->J0.transpose() * jj->lambda);
                }
                else
                {
                    b -= JMinv * (jj->J1.transpose() * jj->lambda);
                }
            }
        }
    }

    // Solve the Boxed LCP problem for a single contact and isotropic Coulomb friction.
    // The solution vector, @a x, contains the impulse the non-interpenetration constraint in x(0), and
    // the friction constraints in x(1) and x(2)
    // 
    // The solution is projected to the lower and upper bounds imposed by the box model.
    // 
    // Inputs: 
    //    x - contains three impulse variables (non-interpenetration + two friction)
    //    b - rhs vector
    //    mu - the friction coefficient
    static inline void solveContact(const Eigen::Matrix3f& A, const Eigen::VectorXf& b, Eigen::VectorXf& x, const float mu)
    {
        // Normal impulse is projected to [0, inf]
        //
        x(0) = (b(0) - A(0, 1) * x(1) - A(0, 2) * x(2)) / (A(0, 0) + 1e-3f);
        if (x(0) < 0.0f) x(0) = 0.0f;

        // Next, friction impulses are projected to [-mu * x(0), mu * x(1)]
        //
        const float lowerx = -mu * x(0);
        const float upperx = mu * x(0);
        x(1) = (b(1) - A(1, 0) * x(0) - A(1, 2) * x(2)) / A(1, 1);
        if (x(1) < lowerx) x(1) = lowerx;
        else if (x(1) > upperx) x(1) = upperx;

        x(2) = (b(2) - A(2, 0) * x(0) - A(2, 1) * x(1)) / A(2, 2);
        if (x(2) < lowerx) x(2) = lowerx;
        else if (x(2) > upperx) x(2) = upperx;

    }

    static inline void solveJoint(const Eigen::LDLT<Eigen::MatrixXf>& LLT, const Eigen::VectorXf& b, Eigen::VectorXf& x)
    {
        x = LLT.solve(b);
    }
}

SolverBoxPGS::SolverBoxPGS(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{

}

void SolverBoxPGS::solve(float h)
{
    std::vector<Contact*>& contacts = m_rigidBodySystem->getContacts();
    std::vector<Joint*>& joints = m_rigidBodySystem->getJoints();
    const int numContacts = contacts.size();
    const int numJoints = joints.size();
    const int N = numJoints + numContacts;

    // Build diagonal matrices of bilateral joints
    std::vector<Eigen::LDLT<Eigen::MatrixXf>> LLTjointii;
    if (numJoints > 0)
    {
        // Build diagonal matrices
        LLTjointii.resize(numJoints);
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            const int dim = j->lambda.rows();
            const float eps = 1e-5f + 1.0f / (h * h * beta + alpha);

            // Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
            //
            Eigen::MatrixXf A = eps * Eigen::MatrixXf::Identity(dim, dim);

            if (!j->body0->fixed)
            {
                JBlockTranspose JT = j->J0.transpose();
                A += j->J0Minv * JT;
            }
            if (!j->body1->fixed)
            {
                JBlockTranspose JT = j->J1.transpose();
                A += j->J1Minv * JT;
            }

            LLTjointii[i] = A.ldlt();
        }
    }

    // Build array of 3x3 diagonal matrices, one for each contact.
    // 
    std::vector<Eigen::Matrix3f> Acontactii;
    if (numContacts > 0)
    {
        // Build diagonal matrices
        Acontactii.resize(numContacts);
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];

            // Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
            //
            Acontactii[i].setZero(3, 3);

            if (!c->body0->fixed)
            {
                Acontactii[i] += c->J0Minv * c->J0.transpose();
            }
            if (!c->body1->fixed)
            {
                Acontactii[i] += c->J1Minv * c->J1.transpose();
            }
        }
    }

    if( N > 0 )
    {
        std::vector<Eigen::VectorXf> b;
        b.resize(N);


        // Compute the right-hand side vector : 
        //      b = -gamma*phi/h - J*vel - dt*JMinvJT*force
        //
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            buildRHS(j, h, b[i]);
        }

        for(int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            buildRHS(c, h, b[i+numJoints]);
            c->lambda.setZero();
        }

        // PGS main loop.
        // There is no convergence test here.
        // Stop after @a maxIter iterations.
        //
        for(int iter = 0; iter < m_maxIter; ++iter)
        {
            // For each joints, compute an updated value of joints[i]->lambda.
            //
            for (int i = 0; i < numJoints; ++i)
            {
                Joint* j = joints[i];
                const int dim = j->lambda.rows();

                // Initialize current solution as x = b[i]
                Eigen::VectorXf x = b[i];

                accumulateCoupledContactsAndJoints(j, j->J0Minv, j->body0, x);
                accumulateCoupledContactsAndJoints(j, j->J1Minv, j->body1, x);
                solveJoint(LLTjointii[i], x, j->lambda);
            }

            // For each contact, compute an updated value of contacts[i]->lambda
            //      using matrix-free pseudo-code provided in the course notes.
            //
            for(int i = 0; i < numContacts; ++i)
            {
                Contact* c = contacts[i];

                // Initialize current solution as x = b[i]
                Eigen::VectorXf x = b[i+numJoints];

                accumulateCoupledContactsAndJoints(c, c->J0Minv, c->body0, x);
                accumulateCoupledContactsAndJoints(c, c->J1Minv, c->body1, x);
                solveContact(Acontactii[i], x, c->lambda, c->mu);
            }
        }
    }
}


