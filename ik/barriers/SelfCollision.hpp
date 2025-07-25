#pragma once

#include <Eigen/Core>
#include <pinocchio/collision/collision.hpp>

#include "ik/Barrier.hpp"

namespace ik {

/**
 * @brief Representation of a barrier that penalises the solver if close to a
 * given limit
 *
 */
class SelfCollisionBarrier : public BarrierAbstract {
   public:
    using Vector = typename BarrierAbstract::Vector;
    using Matrix = typename BarrierAbstract::Matrix;

    SelfCollisionBarrier(const Configuration &cfg,
                         const Size &max_collisions = 1);

    void setMinimumDistance(const Real &distance) { min_distance_ = distance; }
    const Real &getMinimumDistance() const { return min_distance_; }

    void computeBarrier(const Configuration &cfg,
                        Eigen::Ref<Vector> barrier) override;

    void computeJacobian(const Configuration &cfg,
                         Eigen::Ref<Matrix> jacobian) override;

   protected:
   private:
    Size max_collisions_;
    double min_distance_ = 0.02;
    double safe_displacement_limit_ = 1.0;

    std::vector<Size> closest_indices_;
};

}  // namespace ik