/**
 * @file dls.hpp
 * @author Damian Abood (damian.abood@sydney.edu.au)
 * @brief Damped-Least Squares approach
 * @version 0.1
 * @date 2024-11-21
 *
 *
 */
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "ik/problem.hpp"
#include "ik/visitor.hpp"

namespace ik {

/**
 * @brief Dampled least-squares parameters
 *
 */
struct dls_parameters : public default_solver_parameters {
    double damping = 1e-2;
    // Perform random restart if not converging
    bool random_restart = false;
};

// todo - set this data once
struct dls_data {
    dls_data(const InverseKinematicsProblem &problem) {
        std::size_t e_sz = problem.e_size();
        std::size_t c_sz = problem.c_size();

        q = vector_t::Zero(problem.model().nq);
        dq = vector_t::Zero(problem.model().nv);

        e = vector_t::Zero(e_sz);

        J = matrix_t::Zero(e_sz, problem.model().nv);
        Jc = matrix_t::Zero(c_sz, problem.model().nv);

        JJ = matrix_t::Zero(e_sz, e_sz);

        N = matrix_t::Identity(problem.model().nv, problem.model().nv);
    }

    // Whether the program was successful or not
    bool success = false;

    // Configuration vector iterate
    vector_t q;
    // Tangent-space step direction
    vector_t dq;

    // Hessian approximation for the problem
    matrix_t JJ;

    // Constraint jacobian
    matrix_t Jc;
    // Nullspace projection matrix
    matrix_t N;

    // Task error
    vector_t e;
    // Task jacobian
    matrix_t J;
};

/**
 * @brief Damped least-squares solver information
 *
 */
struct dls_info {
    bool success = false;
    int iterations = 0;
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
 * @param data Data associated with the solver, such as matrices and information
 * regarding termination.
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
inline vector_t dls(
    InverseKinematicsProblem &problem, const vector_t &q0, dls_data &data,
    const inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
    const dls_parameters &p = dls_parameters()) {
    data.q = q0;
    data_t model_data = pinocchio::Data(problem.model());

    // todo - if limited convergence, try random walk

    auto tasks = problem.get_all_tasks();
    auto constraints = problem.get_all_constraints();

    // Perform iterations
    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Update model
        pinocchio::framesForwardKinematics(problem.model(), model_data, data.q);
        pinocchio::computeJointJacobians(problem.model(), model_data);
        if (problem.get_centre_of_mass_task())
            pinocchio::jacobianCenterOfMass(problem.model(), model_data, data.q,
                                            false);

        std::size_t cnt = 0;
        for (auto &task : tasks) {
            // Get references
            vector_ref_t ei = data.e.middleRows(cnt, task->dimension());
            matrix_ref_t Ji = data.J.middleRows(cnt, task->dimension());
            // Compute task error and jacobians
            task->compute_error(problem.model(), model_data, ei);
            task->compute_jacobian(problem.model(), model_data, Ji);
            // Weight tasks
            ei = ei.cwiseProduct(task->weighting());
            Ji = task->weighting().asDiagonal() * Ji;
            // Move to next task
            cnt += task->dimension();
        }

        // todo - add constraints here
        cnt = 0;
        for (auto &constraint : constraints) {
            // Get references
            matrix_ref_t Ji = data.Jc.middleRows(cnt, constraint->dimension());
            // Compute constraint error and jacobians
            constraint->compute_jacobian(problem.model(), model_data, Ji);
            // Move to next constraint
            cnt += constraint->dimension();
        }

        // Estimate hessian of cost
        data.JJ.noalias() = data.J * data.J.transpose();
        data.JJ.diagonal().array() += p.damping * p.damping;

        // Project into constraint null-space
        data.N.setIdentity();
        if (data.Jc.rows() > 0) {
            data.N -=
                data.Jc.completeOrthogonalDecomposition().pseudoInverse() *
                data.Jc;
        }

        // Compute step direction
        data.dq = -data.N * (data.J.transpose() * data.JJ.ldlt().solve(data.e));

        VLOG(10) << "dls: it = " << i;
        VLOG(10) << "dls: e = " << data.e.transpose();
        VLOG(10) << "dls: q = " << data.q.transpose();
        VLOG(10) << "dls: dq = " << data.dq.transpose();
        VLOG(15) << "dls: J = " << data.J;

        if (visitor.should_stop(problem, data.e, data.dq)) {
            data.success = true;
            return data.q;
        }

        // Take a step
        data.q = pinocchio::integrate(problem.model(), data.q,
                                      p.step_length * data.dq);

        // Clip joints if any exceed bounds
        apply_joint_clipping(problem.model(), data.q);

        // If issues, perform random restart
    }

    data.success = false;
    return data.q;
    // If it didn't converge, try a random restart?
}

}  // namespace ik