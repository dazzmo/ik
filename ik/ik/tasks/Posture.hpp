#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/constraint.hpp"
#include "ik/task.hpp"

namespace ik {

/**
 * @class PostureTask
 * @brief This class inherits from `Task` and provides the functionality to
 * define a task that penalises deviation from a nominal pose, useful for
 * regularising the system and minimising erratic final poses.
 *
 */
class PostureTask : public Task<Eigen::VectorXd> {
   public:
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     */
    PostureTask(const Configuration &cfg) : Task<Eigen::VectorXd>(cfg.nv()) {}

    void setTargetFromConfiguration() { this->setTarget(cfg.configuration()); }

    /**
     * @brief Computes the task error between the current and target posture
     * configurations.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param e The vector to store the computed error.
     */
    void computeError(const Configuration &cfg, Eigen::Ref<VectorX> e) override {
        e = pinocchio::difference(cfg.configuration(), this->getTarget())
    }

    /**
     * @brief Computes the task Jacobian matrix.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param jac The matrix to store the computed Jacobian.
     */
    void compute_jacobian(const Configuration &cfg, Eigen::Ref<Matrix> jac) override {
        jac.setIdentity();
    }

   protected:
};

}  // namespace ik