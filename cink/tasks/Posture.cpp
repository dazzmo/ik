#include "cink/tasks/Posture.hpp"

namespace cink {

void PostureTask::setTargetFromConfiguration(const Configuration &cfg) {
    this->setTarget(cfg.configuration());
}

void PostureTask::computeError(const Configuration &cfg, Eigen::Ref<Vector> e) {
    if (cfg.hasRootJoint()) {
        e = pinocchio::difference(cfg.model(), cfg.configuration(),
                                  this->getTarget())
                .bottomRows(cfg.nv() - 6);
    } else {
        e = pinocchio::difference(cfg.model(), cfg.configuration(),
                                  this->getTarget());
    }
}

void PostureTask::computeJacobian(const Configuration &cfg,
                                  Eigen::Ref<Matrix> jac) {
    jac.setIdentity();
}

}  // namespace cink