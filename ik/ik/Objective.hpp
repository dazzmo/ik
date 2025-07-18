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
class ObjectiveAbstract {
   public:
    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;

    const Vector &getWeighting() const { return weighting_; }

    virtual void computeObjective(const Configuration &cfg,
                                  Eigen::Ref<Vector> e) = 0;
    virtual void computeGradient(const Configuration &cfg,
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

        // Minimise the gravito-inertial torques
        Vector G;
        // Express this in terms of velocities?
        // c(q) = G^T G
        // \dot c = dc/dq dq/dt = ...
        // ||J v ||
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