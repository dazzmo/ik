#pragma once
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

#include "ik/data.hpp"
#include "ik/problem.hpp"
#include "ik/visitor.hpp"

namespace ik {

struct pik_parameters {
    int max_iterations = 100;
    double damping = 1e-2;
    double step_length = 1.0;
    double max_time = 1.0;
};

Eigen::MatrixXd damp_pseudoinverse(const Eigen::Ref<const Eigen::MatrixXd> &M,
                                   const double &lambda);

struct pik_data : public problem_data {
    pik_data(const InverseKinematicsProblem &problem) : problem_data(problem) {
        P = matrix_t::Identity(problem.model().nv, problem.model().nv);
        lambda.assign(problem.max_priority_level() + 1, 1.0);

        da = vector_t::Zero(problem.model().nv);

        // Check if first joint is floating base
        // Create indices
        free_joint_indices.reserve(problem.model().nv);
        locked_joint_indices.reserve(problem.model().nv);
    }

    // Projection matrix
    matrix_t P;

    vector_t da;

    // Damping factors for each priority level
    std::vector<double> lambda;

    std::vector<std::size_t> free_joint_indices;
    std::vector<std::size_t> locked_joint_indices;
};

/**
 * @brief Priority based inverse kinematics
 *
 * @param ik
 * @param q0
 * @param visitor
 * @param p
 */
vector_t pik(
    InverseKinematicsProblem &problem, const vector_t &q0, pik_data &data,
    const inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
    const pik_parameters &p = pik_parameters());

}  // namespace ik