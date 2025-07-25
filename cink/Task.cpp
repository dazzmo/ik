#include "cink/Task.hpp"

namespace cink {

/**
 * @brief Representation of a task within an inverse-kinematics context, such as
 * maintaining a frame position, orientation or pose. This could also be related
 * to retaining a nominal joint configuration or a centre of mass position.
 *
 */

TaskAbstract::Vector TaskAbstract::computeError(const Configuration &cfg) {
    Vector e = Vector::Zero(this->getDimension());
    computeError(cfg, e);
    return e;
}

TaskAbstract::Matrix TaskAbstract::computeJacobian(const Configuration &cfg) {
    Matrix J = Matrix::Zero(this->getDimension(), cfg.nv());
    computeJacobian(cfg, J);
    return J;
}

void TaskAbstract::addToQPObjective(const Configuration &cfg,
                                    Eigen::Ref<Matrix> H,
                                    Eigen::Ref<Vector> g) {
    Matrix W = this->getWeighting().asDiagonal();
    Matrix J = computeJacobian(cfg);
    Vector e = computeError(cfg);
    Vector We = W * e;

    H += J.transpose() * W * J;
    H.diagonal().array() += getLevenbergMarquardtDamping() * We.dot(We);
    g += J.transpose() * W * e;
}

void TaskAbstract::computeQPObjective(const Configuration &cfg,
                                      Eigen::Ref<Matrix> H,
                                      Eigen::Ref<Vector> g) {
    addToQPObjective(cfg, H, g);
}

}  // namespace ik