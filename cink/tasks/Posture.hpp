#pragma once

#include "cink/Task.hpp"

namespace cink {

/**
 * @class PostureTask
 * @brief This class inherits from `Task` and provides the functionality to
 * define a task that penalises deviation from a nominal pose, useful for
 * regularising the system and minimising erratic final poses.
 *
 */
class PostureTask : public Task<Eigen::VectorXd> {
   public:
    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     */
    PostureTask(const Configuration &cfg)
        : Task<Eigen::VectorXd>(cfg.hasRootJoint() ? cfg.nv() - 6 : cfg.nv()) {}

    void setTargetFromConfiguration(const Configuration &cfg);

    /**
     * @brief Computes the task error between the current and target posture
     * configurations.
     *
     * @param cfg Configuration of the robot
     * @param e The vector to store the computed error.
     */
    void computeError(const Configuration &cfg, Eigen::Ref<Vector> e) override;

    /**
     * @brief Computes the task Jacobian matrix.
     *
     * @param cfg Configuration of the robot
     * @param jac The matrix to store the computed Jacobian.
     */
    void computeJacobian(const Configuration &cfg,
                         Eigen::Ref<Matrix> jac) override;

   protected:
};

}  // namespace cink