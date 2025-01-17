#include "ik/pik.hpp"

namespace ik {

Eigen::MatrixXd damp_pseudoinverse(const Eigen::Ref<const Eigen::MatrixXd> &M,
                                   const double &lambda) {
    // Compute SVD
    auto svd = Eigen::JacobiSVD<Eigen::MatrixXd>(
        M, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Reconstruct the matrix through SVD
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(M.cols(), M.rows());
    for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
        double sigma = svd.singularValues()[i];
        double scaling_factor = (sigma / (pow(lambda, 2) + pow(sigma, 2)));
        res += scaling_factor * svd.matrixV().col(i) *
               svd.matrixU().col(i).transpose();
    }

    return res;
}

/**
 * @brief Priority based inverse kinematics
 *
 * @param ik
 * @param q0
 * @param visitor
 * @param p
 */
vector_t pik(InverseKinematicsProblem &problem, const vector_t &q0,
             pik_data &data, const inverse_kinematics_visitor &visitor,
             const pik_parameters &p) {
        data.q = q0;

    vector_t de_bar;
    matrix_t Jbar;

    for (index_t i = 0; i < p.max_iterations; ++i) {
        // Updata all program data
        evaluate_problem_data(problem, data);

        // Initialisation
        data.P.setIdentity();
        data.dq.setZero();

        for (index_t i = 0; i <= problem.max_priority_level(); ++i) {
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