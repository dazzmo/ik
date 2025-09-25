#pragma once
#include "cink/Limit.hpp"

namespace cink {

/**
 * @brief Enforces the constraint q_{\min} <= q \oplus \delta q = \dot{q} \Delta
 * t <= q_{\max}
 *
 */
class ConfigurationLimit : public LimitAbstract {
    using Matrix = typename LimitAbstract::Matrix;
    using Vector = typename LimitAbstract::Vector;

   public:
    ConfigurationLimit(const Configuration &cfg);


    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubA,
                              const Real &dt) override;
   private:
    Matrix projection_;
    /// @brief Indices of the velocity vector that have configuration limits
    std::vector<Eigen::Index> indices_;
};

}  // namespace ik