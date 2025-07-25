#pragma once

#include <Eigen/Core>

#include "cink/Configuration.hpp"
#include "cink/Types.hpp"

namespace cink {

/**
 * @brief Representation of a task within an inverse-kinematics context, such as
 * maintaining a frame position, orientation or pose. This could also be related
 * to retaining a nominal joint configuration or a centre of mass position.
 *
 */
class ObjectiveAbstract {
   public:
    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;

    const Real &getWeighting() const { return weighting_; }

    virtual void addToQPObjective(const Configuration &cfg,
                                  Eigen::Ref<Matrix> H,
                                  Eigen::Ref<Vector> g) = 0;

    void computeQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                            Eigen::Ref<Vector> g) {
        addToQPObjective(cfg, H, g);
    }

   protected:
    /// @brief The weighting vector
    Real weighting_;

   private:
};

}  // namespace ik