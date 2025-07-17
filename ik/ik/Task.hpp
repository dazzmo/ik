#pragma once

#include <Eigen/Core>

#include "ik/Configuration.hpp"
#include "ik/Types.hpp"
#include "ik/common.hpp"

namespace ik {

/**
 * @brief Representation of a task within an inverse-kinematics context, such as
 * maintaining a frame position, orientation or pose. This could also be related
 * to retaining a nominal joint configuration or a centre of mass position.
 *
 */
class TaskAbstract {
   public:
    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;

    Size getDimension() const { return dimension_; }
    const Vector &getWeighting() const { return weighting_; }

    virtual void computeError(const Configuration &cfg,
                              Eigen::Ref<VectorX> e) = 0;
    virtual void computeJacobian(const Configuration &cfg,
                                 Eigen::Ref<MatrixX> jac) = 0;

   protected:
    TaskAbstract() : dimension_(0), weighting_(Vector::Zero(0)) {}
    TaskAbstract(const Size &dimension)
        : dimension_(dimension), weighting_(Vector::Ones(dimension)) {}

   private:
    Size dimension_;
    Vector weighting_;
};

template <typename _TargetType>
class Task {
   public:
    /// @brief The type of the task's target
    using TargetType = _TargetType;

    void setTarget(const TargetType &target) { target_ = target; }
    const TargetType &getTarget() const { return target_; }

   private:
    TargetType target_;
};

}  // namespace ik