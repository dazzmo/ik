
#include "cink/tasks/AlignAxis.hpp"

namespace cink {

AlignAxisTask::AlignAxisTask(const String &frame, const AlignAxisType &axis)
    : Task<Eigen::Vector3d>(1), frame_(frame), axis_(axis) {}

void AlignAxisTask::computeError(const Configuration &cfg,
                                 Eigen::Ref<Vector> e) {
    // Compute the frame error
    const auto &oMf = cfg.getTransformFrameToWorld(this->frame());
    // Get axis of frame with respect to the reference frame
    const Eigen::Vector3d r = getAxisInWorldFrame(oMf, axis_);

    // Compute alignment error
    e << 1.0 - r.dot(this->getTarget().normalized());
}

double AlignAxisTask::computeAngularError(const Configuration &cfg,
                                          bool degrees) {
    // Compute the frame error
    const auto &oMf = cfg.getTransformFrameToWorld(this->frame());
    // Get axis of frame with respect to the reference frame
    const Eigen::Vector3d r = getAxisInWorldFrame(oMf, axis_);
    // Get the normalised target vector
    const Eigen::Vector3d t = this->getTarget().normalized();
    // Return the error either in degrees or radians (note: r and t are
    // normalised)
    const double angle_rad = acos(r.dot(t));
    constexpr double rad_to_deg = 180.0 / M_PI;
    return degrees ? rad_to_deg * angle_rad : angle_rad;
}

void AlignAxisTask::computeJacobian(const Configuration &cfg,
                                    Eigen::Ref<Matrix> jac) {
    // Compute the frame error
    const auto &oMf = cfg.getTransformFrameToWorld(this->frame());
    const auto &frame_jacobian =
        cfg.getFrameJacobian(cfg.getFrameIndex(this->frame()));
    const Eigen::Vector3d r = getAxisInWorldFrame(oMf, axis_);

    // Create task Jacobian
    jac = -(r.cross(this->getTarget().normalized())).transpose() *
          oMf.rotation() * frame_jacobian.bottomRows(3);
}

Eigen::Vector3d AlignAxisTask::getAxisInWorldFrame(
    const pinocchio::SE3 &oMf, const AlignAxisType &axis) const {
    return oMf.rotation().col(static_cast<Eigen::Index>(axis));
}

}  // namespace cink