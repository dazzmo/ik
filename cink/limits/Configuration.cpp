#include "cink/limits/Configuration.hpp"

namespace cink {

ConfigurationLimit::ConfigurationLimit(const Configuration &cfg,
                                       const Matrix &projection)
    : LimitAbstract(projection.rows()), projection_(projection) {}

void ConfigurationLimit::computeQPConstraints(const Configuration &cfg,
                                              Eigen::Ref<Matrix> A,
                                              Eigen::Ref<Vector> ubA,
                                              Eigen::Ref<Vector> lbA,
                                              const Real &dt) {
    A = dt * projection_;
    ubA =
        getLimitGain() * pinocchio::difference(cfg.model(), cfg.configuration(),
                                               cfg.model().upperPositionLimit);
    lbA =
        getLimitGain() * pinocchio::difference(cfg.model(), cfg.configuration(),
                                               cfg.model().lowerPositionLimit);
}

}  // namespace ik