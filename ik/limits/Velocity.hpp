#include "ik/Limit.hpp"

namespace ik {

/**
 * @brief Enforces the constraint v_{\min} <= v = \delta q / \Delta t = v_{\max}
 *
 */
class VelocityLimit : public LimitAbstract {
    using Matrix = typename LimitAbstract::Matrix;
    using Vector = typename LimitAbstract::Vector;

   public:
    VelocityLimit(const Configuration &cfg, const Matrix &projection,
                  const Vector &ub, const Vector &lb)
        : LimitAbstract(projection.rows()),
          gain_(1.0),
          projection_(projection),
          ub_(ub),
          lb_(lb) {}

    void setLimitGain(const Real &value) { gain_ = value; }

    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA,
                              const Real &dt) override {
        A = projection_;
        ubA = dt * gain_ * ub_;
        lbA = dt * gain_ * lb_;
    }

   private:
    Real gain_;
    Matrix projection_;
    Vector ub_;
    Vector lb_;
};

}  // namespace ik