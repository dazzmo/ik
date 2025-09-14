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
class TaskAbstract {
   public:
    static constexpr double DEFAULT_TOLERANCE = 1e-3;

    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;

    Size getDimension() const { return dimension_; }

    const Vector &getWeighting() const { return weighting_; }
    void setWeighting(const Vector &weighting) { weighting_ = weighting; }
    void setWeighting(const Real &weighting) {
        weighting_.setConstant(weighting);
    }

    const Real &getTolerance() const { return tolerance_; }
    void setTolerance(const Real &tolerance) { tolerance_ = tolerance; }

    void setLevenbergMarquardtDamping(const Real &value) {
        lm_damping_ = value;
    }
    const Real &getLevenbergMarquardtDamping() const { return lm_damping_; }

    virtual void computeError(const Configuration &cfg,
                              Eigen::Ref<Vector> e) = 0;
    virtual void computeJacobian(const Configuration &cfg,
                                 Eigen::Ref<Matrix> jac) = 0;

    void computeWeightedError(const Configuration &cfg, Eigen::Ref<Vector> e) {
        this->computeError(cfg, e);
        e = getWeighting().cwiseProduct(e);
    }

    Vector computeError(const Configuration &cfg);

    Matrix computeJacobian(const Configuration &cfg);

    /**
     * @brief Adds to an existing quadratic objective of the form \frac{1}{2}
     * x^T H x + g^T x
     *
     * @param cfg
     * @param H
     * @param g
     */
    void addToQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                          Eigen::Ref<Vector> g);

    void computeQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                            Eigen::Ref<Vector> g);

   protected:
    TaskAbstract()
        : dimension_(0),
          weighting_(Vector::Zero(0)),
          tolerance_(Real(0)),
          lm_damping_(Real(0)) {}
    TaskAbstract(const Size &dimension)
        : dimension_(dimension),
          weighting_(Vector::Ones(dimension)),
          tolerance_(DEFAULT_TOLERANCE),
          lm_damping_(Real(0)) {}

    Vector &getWeighting() { return weighting_; }

   private:
    /// @brief Dimension of the error-space for the task
    Size dimension_;
    /// @brief The weighting vector
    Vector weighting_;
    /// @brief The tolerances for the task to be considered satisfied
    Real tolerance_;
    /// @brief Levenburg Marquadt damping factor
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

}  // namespace cink