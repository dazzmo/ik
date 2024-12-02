#pragma once

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>

#include "ik/common.hpp"

namespace ik {

template <class T>
struct task_traits {
    typedef typename T::value_type value_type;
    typedef typename T::index_type index_type;
    typedef typename T::integer_type integer_type;
};

struct task_attributes {};

/**
 * @brief Representation of a task within an inverse-kinematics context, such as
 * maintaining a frame position, orientation or pose. This could also be related
 * to retaining a nominal joint configuration or a centre of mass position.
 *
 * @tparam ValueType Type for the values used within the task.
 * @tparam IndexType Type for indexing variables in arrays.
 * @tparam IntegerType Type for integer values.
 */
template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class TaskTpl {
   public:
    typedef ValueType value_type;
    typedef IndexType index_type;
    typedef IntegerType integer_type;

    TaskTpl() : dimension_(index_type(0)) {}
    TaskTpl(const index_type &dimension) : dimension_(dimension) {
        set_dimension(dimension);
    }

    virtual void compute_error(const model_t &model, data_t &data,
                               vector_ref_t e) = 0;

    virtual void compute_jacobian(const model_t &model, data_t &data,
                                  matrix_ref_t jac) = 0;

    /**
     * @brief Dimension of the task (specifically, the dimension of the error
     * between a target and a state)
     *
     * @return index_type
     */
    index_type dimension() const { return dimension_; }

    vector_t &weighting() { return weighting_; }

    const index_type &priority() const { return priority_; }

   protected:
    /**
     * @brief Set the dimension object
     *
     * @param dimension
     */
    void set_dimension(const index_type &dimension) {
        dimension_ = dimension;
        weighting_ = vector_t::Ones(dimension);
    }

   private:
    // Dimension of the task
    index_type dimension_;
    vector_t weighting_;
    index_type priority_;
};

typedef TaskTpl<double> Task;

}  // namespace ik