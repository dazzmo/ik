#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/constraint.hpp"
#include "ik/task.hpp"

namespace ik {

/**
 * @class PostureTaskTpl
 * @brief This class inherits from `TaskTpl` and provides the functionality to
 * define a task that penalises deviation from a nominal pose, useful for
 * regularising the system and minimising erratic final poses.
 *
 * @tparam ValueType The scalar type used for computations (e.g., `double` or
 * `float`).
 * @tparam IndexType The type used for indexing (default: `std::size_t`).
 * @tparam IntegerType The type used for integer values (default: `int`).
 */
template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class PostureTaskTpl : public TaskTpl<ValueType, IndexType, IntegerType> {
   public:
    typedef pinocchio::SE3 se3_t;
    typedef pinocchio::Motion twist_t;

    typedef TaskTpl<ValueType, IndexType, IntegerType> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     */
    PostureTaskTpl(const model_t &model, const index_type &nj)
        : TaskTpl<ValueType, IndexType, IntegerType>(),
          nj_(nj),
          target(vector_t::Zero(nj)),
          mask(vector_t::Ones(nj)) {
        this->set_dimension(nj);
    }

    /**
     * @brief Factory method to create a shared pointer to a posture task.
     *
     * @param model The Pinocchio model of the robot.
     * @return A shared pointer to the created `PostureTaskTpl` instance.
     */
    static std::shared_ptr<PostureTaskTpl> create(const model_t &model,
                                                  const index_type &nj) {
        return std::make_shared<PostureTaskTpl>(model, nj);
    }

    /**
     * @brief Computes the task error between the current and target posture
     * configurations.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param e The vector to store the computed error.
     */
    void compute_error(const model_t &model, data_t &data,
                       const vector_const_ref_t q, vector_ref_t e) override {
        e = (q.bottomRows(nj_) - target).cwiseProduct(mask);
    }

    /**
     * @brief Computes the task Jacobian matrix.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param jac The matrix to store the computed Jacobian.
     */
    void compute_jacobian(const model_t &model, data_t &data,
                          matrix_ref_t jac) override {
        // Create task Jacobian
        jac.rightCols(nj_).setIdentity();
        // todo - incorporate mask
    }

    /**
     * @brief Target posture for the task, with respect to the specified reference
     * posture for the task
     *
     */
    vector_t target;

    /**
     * @brief Mask vector, where setting components to 0 will not include them
     * for consideration in evaluating the posture task.
     *
     */
    vector_t mask;

   protected:
    index_type nj_;
};

typedef PostureTaskTpl<double> PostureTask;

}  // namespace ik