#pragma once
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

#include "ik/data.hpp"
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
class dls_data : public problem_data {
   public:
    dls_data(const InverseKinematicsProblem &problem) : problem_data(problem) {
        
        index_t e_sz = 0, c_sz = 0;
        for (int i = 0; i <= problem.max_priority_level(); ++i) {
            e_sz += problem.e_size(i);
        }

        c_sz = problem.c_size();

        et = vector_t::Zero(e_sz);
        Jt = matrix_t::Zero(e_sz, problem.model().nv);

        Jc = matrix_t::Zero(c_sz, problem.model().nv);

        JJ = matrix_t::Zero(e_sz, e_sz);

        N = matrix_t::Identity(problem.model().nv, problem.model().nv);
    }

    // Hessian approximation for the problem
    matrix_t JJ;

    // Total task error vector
    vector_t et;
    // Total task jacobian
    matrix_t Jt;
    // Constraint jacobian
    matrix_t Jc;
    // Nullspace projection matrix
    matrix_t N;
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

    // todo - if limited convergence, try random walk
    auto constraints = problem.get_all_constraints();

    // Perform iterations
    for (index_t i = 0; i < p.max_iterations; ++i) {
        // Updata all program data
        evaluate_problem_data(problem, data);

        index_t cnt = 0;
        // Get all priority 0 tasks
        for (index_t p = 0; p <= problem.max_priority_level(); ++p) {
            data.et.middleRows(cnt, data.e[p].rows()) << data.e[p];
            data.Jt.middleRows(cnt, data.J[p].rows()) << data.J[p];
            cnt += data.e[p].rows();
        }

        cnt = 0;
        for (auto &constraint : constraints) {
            // Get references
            matrix_ref_t Ji = data.Jc.middleRows(cnt, constraint->dimension());
            // Compute constraint error and jacobians
            constraint->compute_jacobian(problem.model(), data.model_data, Ji);
            // Move to next constraint
            cnt += constraint->dimension();
        }

        // Compute J J^T
        // todo - create an efficient version of this which doesn't required
        // copying
        data.JJ.noalias() = data.Jt * data.Jt.transpose();
        // Damp
        data.JJ.diagonal().array() += p.damping * p.damping;

        // Project into constraint null-space
        data.N.setIdentity();
        if (data.Jc.rows() > 0) {
            data.N -=
                data.Jc.completeOrthogonalDecomposition().pseudoInverse() *
                data.Jc;
        }

        // Compute step direction
        data.dq =
            -data.N * (data.Jt.transpose() * data.JJ.ldlt().solve(data.et));

        VLOG(10) << "dls: it = " << i;
        VLOG(10) << "dls: e = " << data.et.transpose();
        VLOG(10) << "dls: q = " << data.q.transpose();
        VLOG(10) << "dls: dq = " << data.dq.transpose();
        VLOG(15) << "dls: J = " << data.Jt;

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
}

}  // namespace ik