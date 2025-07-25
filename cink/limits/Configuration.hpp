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
    ConfigurationLimit(const Configuration &cfg, const Matrix &projection);


    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA,
                              const Real &dt) override;
   private:
    Matrix projection_;
};

}  // namespace ik