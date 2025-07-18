#pragma once

#include <Eigen/Core>

#include "ik/Configuration.hpp"
#include "ik/Types.hpp"

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

    void setLevenbergMarquardtDamping(const Real &value) {
        lm_damping_ = value;
    }
    const Real &getLevenbergMarquardtDamping() const { return lm_damping_; }

    virtual void computeError(const Configuration &cfg,
                              Eigen::Ref<Vector> e) = 0;
    virtual void computeJacobian(const Configuration &cfg,
                                 Eigen::Ref<Matrix> jac) = 0;

    Vector computeError(const Configuration &cfg) {
        Vector e = Vector::Zero(this->getDimension());
        computeError(cfg, e);
        return e;
    }

    Matrix computeJacobian(const Configuration &cfg) {
        Matrix J = Matrix::Zero(this->getDimension(), cfg.nv());
        computeJacobian(cfg, J);
        return J;
    }

    void addToQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                          Eigen::Ref<Vector> g) {
        Matrix W = this->getWeighting().asDiagonal();
        Matrix J = computeJacobian(cfg);
        Vector e = computeError(cfg);
        Vector We = W * e;

        H += J.transpose() * W * J;
        H.diagonal().array() += getLevenbergMarquardtDamping() * We.dot(We);
        g += J.transpose() * W * e;
    }

    void computeQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                            Eigen::Ref<Vector> g) {
        addToQPObjective(cfg, H, g);
    }

   protected:
    TaskAbstract() : dimension_(0), weighting_(Vector::Zero(0)) {}
    TaskAbstract(const Size &dimension)
        : dimension_(dimension), weighting_(Vector::Ones(dimension)) {}

    /// @brief The weighting vector
    Vector weighting_;

   private:
    Size dimension_;
    Real lm_damping_;
};

template <typename _TargetType>
class Task : public TaskAbstract {
   public:
    /// @brief The type of the task's target
    using TargetType = _TargetType;

    Task() : TaskAbstract() {}
    Task(const Size &dimension) : TaskAbstract(dimension) {}

    void setTarget(const TargetType &target) { target_ = target; }
    const TargetType &getTarget() const { return target_; }

   private:
    TargetType target_;
};

}  // namespace ik