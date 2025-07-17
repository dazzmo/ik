#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/cost.hpp"

namespace ik {

/**
 * @class GravityTorqueMinimisation
 * @brief This class inherits from `Cost` and provides the functionality to
 * define a cost that penalises torques due to the effects of gravity
 *
 */
class GravityTorqueMinimisation : public Cost {
   public:
    /**
     * @brief Constructor for creating a frame cost.
     *
     * @param model The Pinocchio model of the robot.
     */
    GravityTorqueMinimisation(const model_t &model, const index_t &nj) : Task(), nj_(nj) {}

    /**
     * @brief Factory method to create a shared pointer to a posture cost.
     *
     * @param model The Pinocchio model of the robot.
     * @return A shared pointer to the created `GravityTorqueMinimisation` instance.
     */
    static std::shared_ptr<GravityTorqueMinimisation> create(const model_t &model,
                                               const index_t &nj) {
        return std::make_shared<GravityTorqueMinimisation>(model, nj);
    }

    /**
     * @brief Computes the cost error between the current and target posture
     * configurations.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param e The vector to store the computed error.
     */
    void compute_cost(const model_t &model, data_t &data,
                      const vector_const_ref_t q, number_t &cost) override {
        cost = data.rnea_tau.dot(data.rnea_tau);
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