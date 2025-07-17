#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/constraint.hpp"
#include "ik/task.hpp"

namespace ik {

enum class AlignAxisType { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };

/**
 * @brief Task designed to align a particular axis of an end-effector frame to,
 * irrespective of the other axes of the frame. Appropriate for contact tasks
 * where having the end-effector aligned with the contact normal is essential.
 *
 */
class AlignAxisTask : public Task<Eigen::Vector3d> {
   public:
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     */
    AlignAxisTask(const String &frame, const AlignAxisType &axis)
        : Task<Eigen::Vector3d>(1), frame_(frame), axis_(axis) {}

    /**
     * @brief Computes the tasj error between the current and target frame
     * configurations.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param e The vector to store the computed error.
     */
    void computeError(const Configuration &cfg, Eigen::Ref<Vector> e) override {
        // Compute the frame error
        const auto &oMf = cfg.getTransformFrameToWorld(frame);
        // Get axis of frame with respect to the reference frame
        Eigen::Ref<const Eigen::Vector3d> r =
            oMf.rotation().col(static_cast<Eigen::Index>(axis_));

        // Compute alignment error
        e << 1.0 - r.dot(this->getTarget().normalized());
    }

    /**
     * @brief Computes the task Jacobian matrix.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param jac The matrix to store the computed Jacobian.
     */
    void computeJacobian(const Configuration &cfg,
                         Eigen::Ref<Matrix> jac) override {
        // Compute the frame error
        const auto &oMf = cfg.getTransformFrameToWorld(this->frame());
        const auto &frame_jacobian = cfg.getFrameJacobian(this->frame());

        // Create task Jacobian
        jac = -(oMf.rotation()
                    .col(static_cast<Eigen::Index>(axis_))
                    .cross(this->getTarget().normalized()))
                   .transpose() *
              oMf.rotation() * frame_jacobian.bottomRows(3);
    }

   protected:
    // Axis of the frame we align
    AlignAxisType axis_;

    // Frame name
    String frame_;
};
}  // namespace ik