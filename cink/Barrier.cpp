#include "cink/Barrier.hpp"

namespace cink {

void BarrierAbstract::computeQPConstraints(const Configuration &cfg,
                                           Eigen::Ref<Matrix> A,
                                           Eigen::Ref<Vector> lbA,
                                           Eigen::Ref<Vector> ubA,
                                           const Real &dt) {
    Vector b = Vector::Zero(this->dimension_);
    computeBarrier(cfg, b);
    computeJacobian(cfg, A);
    A /= dt;
    lbA = gain_ * b;
    ubA = gain_ * b;
}

}  // namespace ik