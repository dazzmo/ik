/**
 * @file pik.hpp
 * @author your name (you@domain.com)
 * @brief Priority based inverse kinematics approach
 * https://www.cimat.mx/~cesteves/cursos/animation/pdf/BaerlocherBoulic.pdf
 * @version 0.1
 * @date 2024-11-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "ik/problem.hpp"
#include "ik/visitor.hpp"

namespace ik {

struct pik_parameters {
    int max_iterations = 100;
    double damping = 1e-2;
    double step_length = 1.0;
    double max_time = 1.0;
};

class FrameTask : public FrameTask {
   public:
    std::size_t priority = 0;
};

// todo - priority version

void damp_svd(Eigen::JacobiSVD<Eigen::MatrixXd> &svd, const double &lambda) {
    for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
        double sigma = svd.singularValues()[i];
        svd.singularValues()[i] = (pow(lambda, 2) + pow(sigma, 2)) / sigma;
    }
}

/**
 * @brief Priority based inverse kinematics
 *
 * @param ik
 * @param q0
 * @param visitor
 * @param p
 */
void pik(InverseKinematicsProblem &problem, const vector_t &q0,
         inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
         const pik_parameters &p = pik_parameters()) {
    // Parameters we can decide on later
    std::size_t sz = problem.e_size();

    // Initialise jacobian
    vector_t q = q0, dq(problem.model.nv), e(sz);
    matrix_t J(sz, problem.model.nv), JJ(sz, sz);

    data_t data = pinocchio::Data(problem.model);

    // todo - if limited convergence, try random walk

    // Organise tasks into their priorities
    std::vector<std::vector<std::shared_ptr<Task>>> tasks;

    // Recursively compute priority projection
    for (std::size_t priority = 0; priority < 10; ++priority) {
        // Evaluate error and jacobian
        for (auto &task : problem.get_all_tasks()) {
            if (task->priority() == priority) {
                tasks[priority].push_back(task);
            }
        }
    }

    // Perform iterations
    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Update model
        pinocchio::framesForwardKinematics(problem.model, data, q);
        pinocchio::computeJointJacobians(problem.model, data);
        if (problem.get_centre_of_mass_task())
            pinocchio::jacobianCenterOfMass(problem.model, data, q, false);

        matrix_t P, J;
        vector_t dx;
        P.setIdentity();
        // Compute priority levels
        for (std::size_t priority = 0; i < 10; ++i) {
            std::size_t sz = 0;
            for (auto &task : tasks[priority]) {
                sz += task->dimension();
            }

            // Establish Jacobian and error
            vector_t e(sz);
            matrix_t J(sz, problem.model.nv);

            // Evaluate e and J
            std::size_t cnt = 0;
            for (auto &task : tasks[priority]) {
                // Get references
                Eigen::Ref<vector_t> ei =
                    e.middleRows(cnt, task->dimension());
                Eigen::Ref<matrix_t> Ji =
                    J.middleRows(cnt, task->dimension());
                // Compute task error and jacobians
                task->compute_error(problem.model, data, ei);
                task->compute_jacobian(problem.model, data, Ji);
                // Weight tasks
                ei = ei.cwiseProduct(task->weighting());
                Ji = task->weighting().asDiagonal() * Ji;
                // Move to next task
                cnt += task->dimension();
            }

            // Compensate task error
            vector_t e_hat = e - J * dq;
            J = J * P;

            dx += J * de;

            // Compute SVD of Ji
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(
                Ji, Eigen::ComputeThinU | Eigen::ComputeThinV);

            // Restrict Jacobian to nullspace of all higher priorities

            // Compute projection operator to next priority
            P = P - J.completeOrthogonalDecomposition().pseudoInverse() * J;
        }

        // dq = dq + P * da;

        // Other tasks ...

        // dq = dq + P * ;
    }
}

}  // namespace ik