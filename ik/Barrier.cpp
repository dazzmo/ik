#include "ik/Barrier.hpp"

namespace ik {


    void BarrierAbstract::computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
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


}  // namespace ik