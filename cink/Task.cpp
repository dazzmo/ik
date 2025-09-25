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
    const Matrix W = this->getWeighting().asDiagonal();
    const Matrix J = computeJacobian(cfg);
    const Vector e = computeError(cfg);
    const Vector We = W * e;
    const Matrix WJ = W * J;

    H += this->getGain() * WJ.transpose() * WJ;
    H.diagonal().array() += getLevenbergMarquardtDamping() * We.dot(We);
    g += this->getGain() * WJ.transpose() * We;
}

void TaskAbstract::computeQPObjective(const Configuration &cfg,
                                      Eigen::Ref<Matrix> H,
                                      Eigen::Ref<Vector> g) {
    addToQPObjective(cfg, H, g);
}

void TaskAbstract::computeQPConstraints(const Configuration &cfg,
                                       Eigen::Ref<Matrix> A,
                                       Eigen::Ref<Vector> lbA,
                                       Eigen::Ref<Vector> ubA) {
    const Matrix J = this->getGain() * computeJacobian(cfg);
    const Vector e = this->getGain() * computeError(cfg);

    A = J;
    lbA = -e;
    ubA = -e;
}

}  // namespace cink