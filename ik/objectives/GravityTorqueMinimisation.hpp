#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

#include "ik/Objective.hpp"

namespace ik {

/**
 * @class GravityTorqueMinimisation
 * @brief This class inherits from `Cost` and provides the functionality to
 * define a cost that penalises torques due to the effects of gravity
 *
 */
class GravityTorqueMinimisation : public ObjectiveAbstract {
   public:
    void addToQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                          Eigen::Ref<Vector> g) override {
        /// Taylor approximation of potential term
        /// G \approx G(q_0) + dG/dq (q - q_0)
        Matrix J;
        pinocchio::computeGeneralizedGravityDerivatives(cfg.model(), cfg.data(),
                                                        cfg.configuration(), J);
    }

    void computeQPObjective(const Configuration &cfg, Eigen::Ref<Matrix> H,
                            Eigen::Ref<Vector> g) {
        addToQPObjective(cfg, H, g);
    }

    /**
     * @brief Computes the cost Jacobian matrix.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param jac The matrix to store the computed Jacobian.
     */
    void compute_gradient(const model_t &model, data_t &data,
                          vector_ref_t grd) override {
        grd = data.dtau_q;
    }

   protected:
};

}  // namespace ik