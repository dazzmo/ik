#include "ik/tasks/CentreOfMass.hpp"

namespace ik {

void CentreOfMassTask::setTargetFromConfiguration(const Configuration &cfg) {
    this->setTarget(cfg.getCentreOfMass());
}

void CentreOfMassTask::computeError(const Configuration &cfg,
                                    Eigen::Ref<Vector> e) {
    e = cfg.data().com[0] - this->getTarget();
}

void CentreOfMassTask::computeJacobian(const Configuration &cfg,
                                       Eigen::Ref<Matrix> jacobian) {
    jacobian = cfg.computeCentreOfMassJacobian();
}

}  // namespace ik