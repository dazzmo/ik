#include "cink/tasks/Frame.hpp"

#include <iostream>

namespace cink {

void FrameTask::setPositionMask(bool x, bool y, bool z) {
    mask_[0] = x ? 1.0 : 0.0;
    mask_[1] = y ? 1.0 : 0.0;
    mask_[2] = z ? 1.0 : 0.0;
}

void FrameTask::setOrientationMask(bool x, bool y, bool z) {
    mask_[3] = x ? 1.0 : 0.0;
    mask_[4] = y ? 1.0 : 0.0;
    mask_[5] = z ? 1.0 : 0.0;
}

void FrameTask::setPositionCost(const Eigen::Vector3<Real>& cost) {
    Eigen::VectorXd w = this->getWeighting();
    w.topRows<3>() = cost;
    this->setWeighting(w);
}

void FrameTask::setPositionCost(const Real& cost) {
    Eigen::VectorXd w = this->getWeighting();
    w.topRows<3>().setConstant(cost);
    this->setWeighting(w);
}

void FrameTask::setOrientationCost(const Eigen::Vector3<Real>& cost) {
    Eigen::VectorXd w = this->getWeighting();
    w.bottomRows<3>() = cost;
    this->setWeighting(w);
}

void FrameTask::setOrientationCost(const Real& cost) {
    Eigen::VectorXd w = this->getWeighting();
    w.bottomRows<3>().setConstant(cost);
    this->setWeighting(w);
}

void FrameTask::computeError(const Configuration& cfg, Eigen::Ref<Vector> e) {
    const auto& oMf = cfg.getTransformFrameToWorld(this->frame());
    // Target to World
    auto oMt = this->getTarget();
    // Target to Frame
    auto fMt = oMf.actInv(oMt);
    // Compute error between target frame and the current frame of the
    // system
    e = pinocchio::log6(fMt).toVector();
}

void FrameTask::computeJacobian(const Configuration& cfg,
                                Eigen::Ref<Matrix> J) {
    // Frame to World
    const auto& oMf = cfg.getTransformFrameToWorld(this->frame());
    // Target to World
    auto oMt = this->getTarget();
    // Frame to Target
    auto tMf = oMt.actInv(oMf);

    // Construct jacobian of the logarithm map
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(tMf, Jlog);

    // Compute Jacobian of end-effector in local frame
    J = (-Jlog * cfg.getFrameJacobian(cfg.model().getFrameId(this->frame())));
    // Perform masking
    J = J;
}

void FrameTask::setTargetFromConfiguration(const Configuration& cfg) {
    this->setTarget(cfg.getTransformFrameToWorld(this->frame()));
}

}  // namespace cink