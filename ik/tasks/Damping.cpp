
#include "ik/tasks/Damping.hpp"

namespace ik {

DampingTask::DampingTask(const Configuration &cfg)
    : Task<Eigen::VectorXd>(cfg.nv()) {}

void DampingTask::computeError(const Configuration &cfg, Eigen::Ref<Vector> e) {
    e.setZero();
}

void DampingTask::computeJacobian(const Configuration &cfg,
                                  Eigen::Ref<Matrix> jac) {
    jac.setIdentity();
}

}  // namespace ik