#include "cink/tasks/Frame.hpp"

#include <iostream>

namespace cink {

void FrameTask::setPositionCost(const Eigen::Vector3<Real> &cost) {
    this->getWeighting().topRows(3) = cost;
}

void FrameTask::setPositionCost(const Real &cost) {
    this->getWeighting().topRows(3).setConstant(cost);
}

void FrameTask::setOrientationCost(const Eigen::Vector3<Real> &cost) {
    this->getWeighting().bottomRows(3) = cost;
}
void FrameTask::setOrientationCost(const Real &cost) {
    this->getWeighting().bottomRows(3).setConstant(cost);
}

void FrameTask::computeError(const Configuration &cfg, Eigen::Ref<Vector> e) {
    const auto &oMf = cfg.getTransformFrameToWorld(this->frame());
    // Target to World
    auto oMt = this->getTarget();
    // Target to Frame
    auto fMt = oMf.actInv(oMt);
    // Compute error between target frame and the current frame of the
    // system
    e = pinocchio::log6(fMt).toVector();
}

void FrameTask::computeJacobian(const Configuration &cfg,
                                Eigen::Ref<Matrix> J) {
    // Frame to World
    const auto &oMf = cfg.getTransformFrameToWorld(this->frame());
    // Target to World
    auto oMt = this->getTarget();
    // Frame to Target
    auto tMf = oMt.actInv(oMf);

    // Construct jacobian of the logarithm map
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(tMf, Jlog);
    // Compute Jacobian of end-effector in local frame
    J = -Jlog * cfg.getFrameJacobian(cfg.model().getFrameId(this->frame()));
}

void FrameTask::setTargetFromConfiguration(const Configuration &cfg) {
    this->setTarget(cfg.getTransformFrameToWorld(this->frame()));
}

}  // namespace ik