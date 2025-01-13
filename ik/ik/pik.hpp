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

#include "ik/problem.hpp"
#include "ik/data.hpp"
#include "ik/visitor.hpp"

namespace ik {

struct pik_parameters {
    int max_iterations = 100;
    double damping = 1e-2;
    double step_length = 1.0;
    double max_time = 1.0;
};

Eigen::MatrixXd damp_pseudoinverse(const Eigen::Ref<const Eigen::MatrixXd> &M,
                                   const double &lambda) {
    // Compute SVD
    auto svd = Eigen::JacobiSVD<Eigen::MatrixXd>(
        M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Reconstruct the matrix through SVD
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(M.cols(), M.rows());
    for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
        double sigma = svd.singularValues()[i];
        double scaling_factor = (sigma / (pow(lambda, 2) + pow(sigma, 2)));
        res += scaling_factor * V.col(i) * U.col(i).transpose();
    }

    return res;
}

struct pik_data : public problem_data {
    pik_data(const InverseKinematicsProblem &problem) : problem_data(problem) {
        P = matrix_t::Identity(problem.model().nv, problem.model().nv);
        lambda.assign(problem.max_priority_level() + 1, 1e-6);

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
inline vector_t pik(
    InverseKinematicsProblem &problem, const vector_t &q0, pik_data &data,
    const inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
    const pik_parameters &p = pik_parameters()) {
    data.q = q0;

    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Updata all program data
        evaluate_problem_data(problem, data);

        vector_t de_bar;
        matrix_t Jbar;

        // Initialisation
        data.P.setIdentity();
        data.dq.setZero();


        for (int i = 0; i <= problem.max_priority_level(); ++i) {
            // Remove contribution of task
            de_bar = data.e[i] - data.J[i] * data.dq;
            //
            Jbar = data.J[i] * data.P;

            // Augment step
            data.dq =
                data.dq - damp_pseudoinverse(Jbar, data.lambda[i]) * de_bar;

            // Update projection
            data.P =
                data.P -
                Jbar.completeOrthogonalDecomposition().pseudoInverse() * Jbar;
        }

        // Compute direction
        data.dq = data.dq + data.P * data.da;

        if (visitor.should_stop(problem, data.e, data.dq)) {
            data.success = true;
            return data.q;
        }

        // Take a step
        data.q = pinocchio::integrate(problem.model(), data.q,
                                      p.step_length * data.dq);

        // Clip joints if any exceed bounds
        apply_joint_clipping(problem.model(), data.q);

        // for (const std::size_t &idx : data.free_joint_indices) {
        //     if (data.q[idx] > problem.model().upperPositionLimit[idx]) {
        //         // dx -= Ji * dtheta
        //         data.q[idx] = problem.model().upperPositionLimit[idx];
        //         data.J[priority].col(0).setZero();
        //         // Projection diagonal
        //         // remove index and add to the locked joints
        //     }

        //     if (data.q[idx] < problem.model().lowerPositionLimit[idx]) {
        //     }
        // }

        // If issues, perform random restart
    }

    data.success = false;
    return data.q;
}

}  // namespace ik