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
                  const Vector &ub, const Vector &lb);

    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA,
                              const Real &dt) override;

   private:
    Matrix projection_;
    Vector ub_;
    Vector lb_;
};

}  // namespace ik