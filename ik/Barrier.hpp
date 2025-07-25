#pragma once

#include <Eigen/Core>

#include "ik/Configuration.hpp"
#include "ik/Types.hpp"

namespace ik {

/**
 * @brief Representation of a barrier that penalises the solver if close to a
 * given limit
 *
 */
class BarrierAbstract {
   public:
    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;

    Size getDimension() const { return dimension_; }

    virtual void computeBarrier(const Configuration &cfg,
                                Eigen::Ref<Vector> barrier) {}
    virtual void computeJacobian(const Configuration &cfg,
                                 Eigen::Ref<Matrix> jacobian) {}

    /**
     * @brief Sets the gain of the barrier (i.e. scales the bounds of the
     * barrier constraints)
     *
     * @param gain
     */
    void setLimitGain(const Real &gain) { gain_ = gain; }
    const Real &getLimitGain() const { return gain_; }

    void computeQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                            Eigen::Ref<Vector> g, const Real &dt) {}

    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA,
                              const Real &dt);

   protected:
    BarrierAbstract() : dimension_(0) {}
    BarrierAbstract(const Size &dimension) : dimension_(dimension) {}

   private:
    Real gain_;
    Size dimension_;
};

}  // namespace ik