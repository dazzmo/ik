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

    virtual void computeQPConstraints(const Configuration &cfg,
                                      Eigen::Ref<Matrix> A,
                                      Eigen::Ref<Vector> lbA,
                                      Eigen::Ref<Vector> ubA,
                                      const Real &dt) = 0;

   protected:
    LimitAbstract() : dimension_(0) {}
    LimitAbstract(const Size &dimension) : dimension_(dimension) {}

   private:
    Size dimension_;
};

}  // namespace ik