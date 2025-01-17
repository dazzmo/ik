#pragma once

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>

#include "ik/common.hpp"

namespace ik {

/**
 * @brief Representation of a task within an inverse-kinematics context, such as
 * maintaining a frame position, orientation or pose. This could also be related
 * to retaining a nominal joint configuration or a centre of mass position.
 *
 */

class Task {
   public:
    Task() : dimension_(index_t(0)) {}
    Task(const index_t &dimension) : dimension_(dimension) {
        set_dimension(dimension);
    }

    virtual void compute_error(const model_t &model, data_t &data,
                               const vector_const_ref_t q, vector_ref_t e) = 0;

    virtual void compute_jacobian(const model_t &model, data_t &data,
                                  matrix_ref_t jac) = 0;

    /**
     * @brief Dimension of the task (specifically, the dimension of the error
     * between a target and a state)
     *
     * @return index_t
     */
    index_t dimension() const { return dimension_; }

    vector_t &weighting() { return weighting_; }

   protected:
    /**
     * @brief Set the dimension object
     *
     * @param dimension
     */
    void set_dimension(const index_t &dimension) {
        dimension_ = dimension;
        weighting_ = vector_t::Ones(dimension);
    }

   private:
    // Dimension of the task
    index_t dimension_;
    vector_t weighting_;
};

}  // namespace ik