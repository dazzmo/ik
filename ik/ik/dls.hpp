/**
 * @file dls.hpp
 * @author your name (you@domain.com)
 * @brief Damped-Least Squares approach
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

struct dls_parameters {
    int max_iterations = 100;
    double damping = 1e-2;
    double step_length = 1.0;
    double max_time = 1.0;
    // Perform random restart if not converging
    bool random_restart = false;
};

/**
 * @brief Computes the inverse kinematics solution using the Damped Least
 * Squares (DLS) method.
 *
 * This function iteratively refines the joint configuration to satisfy
 * all tasks defined in the inverse kinematics problem. If convergence
 * is not achieved within the maximum number of iterations, it returns
 * the initial joint configuration or attempts a random restart (if
 * implemented).
 *
 * @param problem An instance of the inverse kinematics problem, containing the
 * model and tasks.
 * @param q0 Initial joint configuration vector.
 * @param visitor A visitor object to handle intermediate states and termination
 * criteria. Default is an empty visitor that performs no special handling.
 * @param p Parameters for the DLS algorithm, including damping factor and
 * iteration limits. Default parameters are provided if not specified.
 *
 * @return The joint configuration that satisfies the tasks, or the initial
 * configuration `q0` if convergence is not achieved.
 *
 * The algorithm performs the following steps:
 * - Updates the forward kinematics and joint Jacobians for the current
 * configuration.
 * - Computes the task errors and Jacobians for all tasks.
 * - Constructs a damped least squares system to estimate the step direction.
 * - Applies the step and updates the joint configuration.
 * - Optionally clips joint values to remain within bounds.
 * - Terminates early if the visitor determines the stopping condition is met.
 *
 * @note If limited convergence occurs, consider adding a random restart
 * mechanism.
 */
vector_t dls(
    InverseKinematicsProblem &problem, const vector_t &q0,
    const inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
    const dls_parameters &p = dls_parameters()) {
    // Parameters we can decide on later
    std::size_t sz = problem.e_size();

    // Initialise jacobian
    vector_t q = q0, dq(problem.model().nv), e(sz);
    matrix_t J(sz, problem.model().nv), JJ(sz, sz);

    data_t data = pinocchio::Data(problem.model());

    // todo - if limited convergence, try random walk

    auto tasks = problem.get_all_tasks();

    // Perform iterations
    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Update model
        pinocchio::framesForwardKinematics(problem.model(), data, q);
        pinocchio::computeJointJacobians(problem.model(), data);
        if (problem.get_centre_of_mass_task())
            pinocchio::jacobianCenterOfMass(problem.model(), data, q, false);

        std::size_t cnt = 0;
        for (auto &task : tasks) {
            // Get references
            vector_ref_t ei = e.middleRows(cnt, task->dimension());
            matrix_ref_t Ji = J.middleRows(cnt, task->dimension());
            // Compute task error and jacobians
            task->compute_error(problem.model(), data, ei);
            task->compute_jacobian(problem.model(), data, Ji);
            // Weight tasks
            ei = ei.cwiseProduct(task->weighting());
            Ji = task->weighting().asDiagonal() * Ji;
            // Move to next task
            cnt += task->dimension();
        }

        // Estimate hessian of cost
        JJ.noalias() = J * J.transpose();
        JJ.diagonal().array() += p.damping * p.damping;
        // Compute step direction
        dq = -J.transpose() * JJ.ldlt().solve(e);

        VLOG(10) << "dls: it = " << i;
        VLOG(10) << "dls: e = " << e.transpose();
        VLOG(10) << "dls: q = " << q.transpose();
        VLOG(10) << "dls: dq = " << dq.transpose();
        VLOG(15) << "dls: J = " << J;

        if (visitor.should_stop(problem, dq)) {
            return q;
        }

        // Take a step
        q = pinocchio::integrate(problem.model(), q, p.step_length * dq);

        // Clip joints if any exceed bounds
        apply_joint_clipping(problem.model(), q);

        // If issues, perform random restart
    }

    return q;
    // If it didn't converge, try a random restart?
}

}  // namespace ik