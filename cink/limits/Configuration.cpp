#include "cink/limits/Configuration.hpp"

#include <iostream>
namespace cink {

ConfigurationLimit::ConfigurationLimit(const Configuration &cfg)
    : LimitAbstract(), indices_({}) {
    // Determine joints that have limits
    const auto condition = (cfg.model().upperPositionLimit.array() < 1e20 &&
                            (cfg.model().upperPositionLimit.array() >
                             cfg.model().lowerPositionLimit.array() + 1e-10));

    // Compute a projection matrix that extracts the necessary limited joints
    indices_.clear();
    for (const auto &joint : cfg.model().joints) {
        if (joint.idx_q() >= 0 &&
            condition.middleRows(joint.idx_q(), joint.nq()).all()) {
            // Add indices to vector
            for (Eigen::Index i = joint.idx_v(); i < joint.idx_v() + joint.nv();
                 ++i) {
                indices_.push_back(i);
            }
        }
    }

    // Create projection matrix
    projection_ = Matrix::Identity(cfg.model().nv, cfg.model().nv);
    projection_ = projection_(indices_, Eigen::all);
    // Set the dimension of the limit
    this->setDimension(indices_.size());
}

void ConfigurationLimit::computeQPConstraints(const Configuration &cfg,
                                              Eigen::Ref<Matrix> A,
                                              Eigen::Ref<Vector> lbA,
                                              Eigen::Ref<Vector> ubA,
                                              const Real &dt) {
    A = dt * projection_;
    ubA = getLimitGain() *
          pinocchio::difference(cfg.model(), cfg.configuration(),
                                cfg.model().upperPositionLimit)(indices_);
    lbA = getLimitGain() *
          pinocchio::difference(cfg.model(), cfg.configuration(),
                                cfg.model().lowerPositionLimit)(indices_);
}

}  // namespace cink