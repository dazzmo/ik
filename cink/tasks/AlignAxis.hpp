#pragma once

#include "cink/Task.hpp"

namespace cink {

enum class AlignAxisType { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };

/**
 * @brief Task designed to align a particular axis of an end-effector frame to,
 * irrespective of the other axes of the frame. Appropriate for contact tasks
 * where having the end-effector aligned with the contact normal is essential.
 *
 */
class AlignAxisTask : public Task<Eigen::Vector3d> {
   public:
    AlignAxisTask(const String &frame, const AlignAxisType &axis);

    const String &frame() const { return frame_; }

    void computeError(const Configuration &cfg, Eigen::Ref<Vector> e) override;

    void computeJacobian(const Configuration &cfg,
                         Eigen::Ref<Matrix> jac) override;

   protected:
    // Frame name
    String frame_;
    // Axis of the frame we align
    AlignAxisType axis_;

    Eigen::Vector3d getAxisInWorldFrame(const pinocchio::SE3 &oMf,
                                        const AlignAxisType &axis) const;
};
}  // namespace ik