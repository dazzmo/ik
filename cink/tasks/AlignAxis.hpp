#pragma once

#include "cink/Task.hpp"

namespace cink {

/**
 * @brief Task designed to align a particular axis of an end-effector frame to,
 * irrespective of the other axes of the frame. Appropriate for contact tasks
 * where having the end-effector aligned with the contact normal is essential.
 *
 * This is based on the `placo` axis_align task, creating a two-dimensional task
 * to achieve axis alignment.
 *
 */
class AlignAxisTask : public Task<Eigen::Vector3d> {
   public:
    AlignAxisTask(const String& frame, const Eigen::Vector3d& axis);

    const String& frame() const { return frame_; }

    /**
     * @brief Computes the angular distance between the target normal and the
     * axis of the frame.
     *
     * @param cfg
     * @param degrees Whether to return the error in degrees (true) or radians
     * (false) (default = false)
     */
    double computeAngularError(const Configuration& cfg, bool degrees = false);

    void computeError(const Configuration& cfg, Eigen::Ref<Vector> e) override;

    void computeJacobian(const Configuration& cfg,
                         Eigen::Ref<Matrix> jac) override;

   protected:
    // Frame name
    String frame_;
    // Axis of the frame we want to align
    Eigen::Vector3d axis_;

    Eigen::Matrix3d axis_frame_{Eigen::Matrix3d::Identity()};

    /**
     * @brief Constructs an orientation matrix such that:
     * - The x-axis is aligned to the axis intended for alignment
     * - The z-axis is parallel to the perpendicular of the plan spanned by the
     * x-axis and the target axis
     * - The y-axis completes the orthogonal basis
     *
     * This transform then maps vectors from this coordinate system to the world
     * frame.
     *
     * @param axis
     * @return Eigen::Matrix3d
     */
    Eigen::Matrix3d createAxisFrame(const Configuration& cfg, const Eigen::Vector3d& axis) const;

};
}  // namespace cink