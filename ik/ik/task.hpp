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

    typedef typename T::error_vector_type error_vector_type;
    typedef typename T::jacobian_matrix_type jacobian_matrix_type;
};

struct task_attributes {};

template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class TaskTpl {
   public:
    typedef ValueType value_type;
    typedef IndexType index_type;
    typedef IntegerType integer_type;

    typedef eigen_vector_tpl_t<ValueType> error_vector_type;
    typedef eigen_matrix_tpl_t<ValueType> jacobian_matrix_type;

    typedef Eigen::Ref<error_vector_type> error_vector_ref_type;
    typedef Eigen::Ref<jacobian_matrix_type> jacobian_matrix_ref_type;

    TaskTpl() : dimension_(index_type(0)) {}
    TaskTpl(const index_type &dimension) : dimension_(dimension) {
        set_dimension(dimension);
    }

    virtual void compute_error(const model_t &model, data_t &data,
                               error_vector_ref_type e) = 0;

    virtual void compute_jacobian(const model_t &model, data_t &data,
                                  jacobian_matrix_ref_type jac) = 0;

    /**
     * @brief Dimension of the task (specifically, the dimension of the error
     * between a target and a state)
     *
     * @return index_type
     */
    index_type dimension() const { return dimension_; }

    eigen_vector_t &weighting() { return weighting_; }

    const index_type &priority() const { return priority_; }

   protected:
    /**
     * @brief Set the dimension object
     *
     * @param dimension
     */
    void set_dimension(const index_type &dimension) {
        dimension_ = dimension;
        weighting_ = eigen_vector_t::Ones(dimension);
    }

   private:
    // Dimension of the task
    index_type dimension_;
    eigen_vector_t weighting_;
    index_type priority_;
};

typedef TaskTpl<double> Task;

}  // namespace ik