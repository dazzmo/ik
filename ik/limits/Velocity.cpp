#include "ik/limits/Velocity.hpp"

namespace ik {

VelocityLimit::VelocityLimit(const Configuration &cfg, const Matrix &projection,
                             const Vector &ub, const Vector &lb)
    : LimitAbstract(projection.rows()),
      projection_(projection),
      ub_(ub),
      lb_(lb) {}

void VelocityLimit::computeQPConstraints(const Configuration &cfg,
                                         Eigen::Ref<Matrix> A,
                                         Eigen::Ref<Vector> ubA,
                                         Eigen::Ref<Vector> lbA,
                                         const Real &dt) {
    A = projection_;
    ubA = dt * getLimitGain() * ub_;
    lbA = dt * getLimitGain() * lb_;
}
}  // namespace ik