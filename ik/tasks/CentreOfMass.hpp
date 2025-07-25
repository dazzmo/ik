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

    void setTargetFromConfiguration(const Configuration &cfg);

    void computeError(const Configuration &cfg, Eigen::Ref<Vector> e) override;

    void computeJacobian(const Configuration &cfg,
                         Eigen::Ref<Matrix> jacobian) override;

   private:
};

}  // namespace ik