#pragma once

#include "ik/Task.hpp"

namespace ik {

/**
 * @class DampingTask
 * @brief A task that penalises the generalised velocity of the system
 */
class DampingTask : public Task<Eigen::VectorXd> {
   public:
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     */
    DampingTask(const Configuration &cfg) : Task<Eigen::VectorXd>(cfg.nv()) {}

    /**
     * @brief Computes the task error between the current and target posture
     * configurations.
     *
     * @param e The vector to store the computed error.
     */
    void computeError(const Configuration &cfg,
                      Eigen::Ref<VectorX> e) override {
        e.setZero();
    }

    /**
     * @brief Computes the task Jacobian matrix.
     *
     * @param jac The matrix to store the computed Jacobian.
     */
    void computeJacobian(const Configuration &cfg,
                         Eigen::Ref<Matrix> jac) override {
        jac.setIdentity();
    }

   protected:
};

}  // namespace ik