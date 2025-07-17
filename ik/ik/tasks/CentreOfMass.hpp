#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/Task.hpp"

namespace ik {

/**
 * @brief Task for the centre of mass location (a 3D point in space).
 *
 */
class CentreOfMassTask : public Task<Eigen::Vector3d> {
   public:
    CentreOfMassTask() : Task<Eigen::Vector3d>(3) {};

    void compute_error(const Configuration &cfg, Eigen::Ref<Vector> e) override {
        e = data.com[0] - this->getTarget();
    }

    void compute_jacobian(const Configuration &cfg, Eigen::Ref<Matrix> jacobian) override {
        jac = 
    }
   private:
};

}  // namespace ik