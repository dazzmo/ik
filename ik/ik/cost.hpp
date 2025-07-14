#pragma once

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>

#include "ik/common.hpp"

namespace ik {

/**
 * @brief Representation of a cost within an inverse-kinematics context.
 *
 */

class Cost {
   public:
    Cost() : weighting_(1.0) {}

    virtual void compute_cost(const model_t &model, data_t &data,
                              const vector_const_ref_t q, number_t &cost) = 0;

    virtual void compute_gradient(const model_t &model, data_t &data,
                                  vector_ref_t grd) = 0;

    number_t &weighting() { return weighting_; }

   protected:
   private:
    // Dimension of the task
    number_t weighting_;
};

}  // namespace ik