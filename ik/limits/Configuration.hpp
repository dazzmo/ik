#include "ik/Limit.hpp"

namespace ik {

/**
 * @brief Enforces the constraint q_{\min} <= q \oplus \delta q = \dot{q} \Delta
 * t <= q_{\max}
 *
 */
class ConfigurationLimit : public LimitAbstract {
    using Matrix = typename LimitAbstract::Matrix;
    using Vector = typename LimitAbstract::Vector;

   public:
    ConfigurationLimit(const Configuration &cfg, const Matrix &projection)
        : LimitAbstract(projection.rows()),
          gain_(1.0),
          projection_(projection) {}

    void setLimitGain(const Real &value) { gain_ = value; }

    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA,
                              const Real &dt) override {
        A = dt * projection_;
        ubA = gain_ * pinocchio::difference(cfg.model(), cfg.configuration(),
                                            cfg.model().upperPositionLimit);
        lbA = gain_ * pinocchio::difference(cfg.model(), cfg.configuration(),
                                            cfg.model().lowerPositionLimit);
    }

   private:
    Real gain_;
    Matrix projection_;
};

}  // namespace ik