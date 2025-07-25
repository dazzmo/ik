#pragma once

#include "cink/Task.hpp"

namespace cink {

/**
 * @class DampingTask
 * @brief A task that penalises the generalised velocity of the system
 */
class DampingTask : public Task<Eigen::VectorXd> {
   public:
   using Vector = Eigen::VectorX<Real>;
   using Matrix = Eigen::MatrixX<Real>;
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     */
    DampingTask(const Configuration &cfg);

    /**
     * @brief Computes the task error between the current and target posture
     * configurations.
     * @param cfg The configuration of the robot
     * @param e The vector to store the computed error.
     */
    void computeError(const Configuration &cfg, Eigen::Ref<Vector> e);

    /**
     * @brief Computes the task Jacobian matrix.
     *
     * @param cfg The configuration of the robot
     * @param jac The matrix to store the computed Jacobian.
     */
    void computeJacobian(const Configuration &cfg, Eigen::Ref<Matrix> jac);

   protected:
};

}  // namespace ik