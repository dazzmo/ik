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

    void computeQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                            Eigen::Ref<Vector> g, const Real &dt) {}

    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> ubA, Eigen::Ref<Vector> lbA,
                              const Real &dt) {
        Vector b = Vector::Zero(this->dimension_);
        Matrix J = Matrix::Zero(this->dimension_, cfg.nv());
        computeBarrier(cfg, b);
        std::cout << "b = " << b << std::endl;
        computeJacobian(cfg, J);
        std::cout << "J = " << J << std::endl;
        A = J / dt;
        lbA = gain_ * b;
        ubA = gain_ * b;
    }

   protected:
    BarrierAbstract() : dimension_(0) {}
    BarrierAbstract(const Size &dimension) : dimension_(dimension) {}

   private:
    Real gain_;
    Size dimension_;
};

}  // namespace ik