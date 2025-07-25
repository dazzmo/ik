#pragma once

#include <Eigen/Core>

#include "ik/Configuration.hpp"
#include "ik/Types.hpp"

namespace ik {

/**
 * @brief Representation of a limit that should be respected
 *
 */
class LimitAbstract {
   public:
    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;

    Size getDimension() const { return dimension_; }

    /**
     * @brief Sets the gain of the limit (i.e. scales the bounds of the limit
     * constraints)
     *
     * @param gain
     */
    void setLimitGain(const Real &gain) { gain_ = gain; }
    const Real &getLimitGain() const { return gain_; }

    virtual void computeQPConstraints(const Configuration &cfg,
                                      Eigen::Ref<Matrix> A,
                                      Eigen::Ref<Vector> lbA,
                                      Eigen::Ref<Vector> ubA,
                                      const Real &dt) = 0;

   protected:
    LimitAbstract() : dimension_(0), gain_(1.0) {}
    LimitAbstract(const Size &dimension) : dimension_(dimension), gain_(1.0) {}

   private:
    Size dimension_;
    Real gain_;
};

}  // namespace ik