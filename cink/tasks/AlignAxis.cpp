
#include "cink/tasks/AlignAxis.hpp"

#include <iostream>
namespace cink {

AlignAxisTask::AlignAxisTask(const String& frame, const Eigen::Vector3d& axis)
    : Task<Eigen::Vector3d>(2), frame_(frame), axis_(axis) {}

Eigen::Matrix3d AlignAxisTask::createAxisFrame(
    const Configuration& cfg, const Eigen::Vector3d& axis) const {
    const auto& oMf = cfg.getTransformFrameToWorld(this->frame());
    Eigen::Matrix3d oRa;
    oRa.col(0) = (oMf.rotation() * axis_).normalized();
    oRa.col(2) = oRa.col(0).cross(this->getTarget()).normalized();
    oRa.col(1) = oRa.col(2).cross(oRa.col(0));
    return oRa;
}

void AlignAxisTask::computeError(const Configuration& cfg,
                                 Eigen::Ref<Vector> e) {
    const auto oRa = createAxisFrame(cfg, axis_);
    // Determine dot product of the axis frame's x-axis with the target world
    // axis
    const double v =
        std::min(1.0, std::max(oRa.col(0).dot(this->getTarget()), -1.0));
    // Compute alignment error
    e << 0.0, -std::acos(v);
}

void AlignAxisTask::computeJacobian(const Configuration& cfg,
                                    Eigen::Ref<Matrix> jac) {
    const auto oRa = createAxisFrame(cfg, axis_);
    const auto& frame_jacobian = cfg.getFrameJacobian(
        cfg.getFrameIndex(this->frame()), pinocchio::WORLD);

    // Create task Jacobian (only enforce in the y and z axes)
    jac = (oRa.transpose() * frame_jacobian.bottomRows(3)).bottomRows<2>();
}

double AlignAxisTask::computeAngularError(const Configuration& cfg,
                                          bool degrees) {
    // Create axis transform
    const auto oRa = createAxisFrame(cfg, axis_);
    // Determine dot product of the axis frame's x-axis with the target world
    // axis
    const double v =
        std::min(1.0, std::max(oRa.col(0).dot(this->getTarget()), -1.0));
    // Compute alignment error
    const double angle_rad = std::acos(v);
    constexpr double rad_to_deg = 180.0 / M_PI;
    return degrees ? rad_to_deg * angle_rad : angle_rad;
}

}  // namespace cink