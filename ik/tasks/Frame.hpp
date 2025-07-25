#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/Task.hpp"

namespace ik {

/**
 * @class FrameTask
 * @brief A template class for defining inverse kinematics tasks for specific
 * frames in a robotic system.
 *
 * This class inherits from `Task` and provides the functionality to define
 * tasks associated with specific frames of a robot (e.g., end-effector
 * positions or orientations).
 */
class FrameTask : public Task<pinocchio::SE3Tpl<Real>> {
   public:
    FrameTask(const String &frame)
        : Task<pinocchio::SE3Tpl<Real>>(6), frame_(frame) {}

    const String &frame() const { return frame_; }

    /**
     * @brief Set the cost of the position error
     * @note This is in [cost] / [m]
     *
     * @param cost
     */
    void setPositionCost(const Eigen::Vector3<Real> &cost);
    /**
     * @copydoc FrameTask::setPositionCost(const Eigen::Vector3<Real> &)
     */
    void setPositionCost(const Real &cost);

    /**
     * @brief Set the cost of the rotation error
     * @note This is in [cost] / [rad]
     *
     * @param cost
     */
    void setOrientationCost(const Eigen::Vector3<Real> &cost);
    void setOrientationCost(const Real &cost);

    /**
     * @brief Computes the error of the frame with respect to the target as an
     * error within the body frame of the frame.
     *
     * @param cfg
     * @param e
     */
    void computeError(const Configuration &cfg, Eigen::Ref<Vector> e) override;

    /**
     * @brief Computes the error of the frame with respect to the target as an
     * error within the body frame of the frame.
     *
     * @param cfg
     * @param e
     */
    void computeJacobian(const Configuration &cfg,
                         Eigen::Ref<Matrix> J) override;

    void setTargetFromConfiguration(const Configuration &cfg);

   private:
    std::string frame_;
};

}  // namespace ik