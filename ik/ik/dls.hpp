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
 * @brief Damped least-squares parameters
 *
 */
struct dls_parameters : public default_solver_parameters {
    number_t damping = 1e-2;
    // Perform random restart if not converging
    bool random_restart = false;
};

/**
 * @brief Damped least-squares (DLS) problem data
 *
 */
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
vector_t dls(
    InverseKinematicsProblem &problem, const vector_t &q0, dls_data &data,
    const inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
    const dls_parameters &p = dls_parameters());

}  // namespace ik