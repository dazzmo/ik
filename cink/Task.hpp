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
    enum Type { OBJECTIVE = 0, CONSTRAINT };

    /// @brief The type of the task, whether it is an objective or a constraint
    Type type_{OBJECTIVE};

   public:
    static constexpr double DEFAULT_TOLERANCE = 1e-3;

    using Vector = Eigen::VectorX<Real>;
    using Matrix = Eigen::MatrixX<Real>;

    /**
     * @brief Get the dimension of the error space of the task.
     *
     * @return Size
     */
    Size getDimension() const { return dimension_; }

    /**
     * @brief The gain of the task, a positive scalar value. A measure of
     * importance of the entire task (not the individual weightings) such that
     * as an objective the cost is J(x) = gain * || task_error ||_W^2. In a
     * constraint context the constraint is active for all gain > 0 and inactive
     * where gain = 0.
     *
     * @return const Real&
     */
    const Real &getGain() const { return gain_; }
    void setGain(const Real &gain) { gain_ = gain; }

    /**
     * @brief Get the weighting vector for the task
     *
     * @return const Vector&
     */
    const Vector &getWeighting() const { return weighting_; }
    void setWeighting(const Vector &weighting) { weighting_ = weighting; }
    void setWeighting(const Real &weighting) {
        weighting_.setConstant(weighting);
    }

    bool isObjective() const { return type_ == OBJECTIVE; }
    bool isConstraint() const { return type_ == CONSTRAINT; }

    void setAsConstraint() { type_ = CONSTRAINT; }
    void setAsObjective() { type_ = OBJECTIVE; }

    /**
     * @brief Get the tolerance of the task such that the task is considered
     * satisfied if ||e||_\infty <= tolerance
     *
     * @return const Real&
     */
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

    /**
     * @brief Computes the constraint such that J\dot q + e = 0 (i.e. -e <= J
     * \dot q <= -e)
     *
     * @param cfg
     * @param A The jacobian block
     * @param lbA
     * @param ubA
     */
    void computeQPConstraints(const Configuration &cfg, Eigen::Ref<Matrix> A,
                              Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubA);

   protected:
    TaskAbstract()
        : dimension_(0),
          gain_(Real(0)),
          weighting_(Vector::Zero(0)),
          tolerance_(Real(0)),
          lm_damping_(Real(0)) {}
    TaskAbstract(const Size &dimension)
        : dimension_(dimension),
          gain_(Real(1)),
          weighting_(Vector::Ones(dimension)),
          tolerance_(DEFAULT_TOLERANCE),
          lm_damping_(Real(0)) {}

   private:
    /// @brief Dimension of the error-space for the task
    Size dimension_;
    /// @brief The gain of the task
    Real gain_;
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