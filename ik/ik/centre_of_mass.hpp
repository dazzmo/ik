#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/constraint.hpp"
#include "ik/task.hpp"

namespace ik {

/**
 * @brief Task for the centre of mass location (a 3D point in space).
 *
 */
class CentreOfMassTask : public Task {
   public:
    CentreOfMassTask(const model_t &model, const std::string &reference_frame)
        : Task(3), reference_frame(reference_frame) {};

    /**
     * @brief Factory method to create a shared pointer to a centre of mass
     * task.
     *
     * @param model The Pinocchio model of the robot.
     * @param reference_frame The name of the reference frame for the centre of
     * mass (default: "universe").
     * @return A shared pointer to the created `CentreOfMassTask` instance.
     */
    static std::shared_ptr<CentreOfMassTask> create(
        const model_t &model, const std::string &reference_frame = "universe") {
        return std::make_shared<CentreOfMassTask>(model, reference_frame);
    }

    void compute_error(const model_t &model, data_t &data,
                       const vector_const_ref_t q, vector_ref_t e) override {
        e = data.oMf[model.getFrameId(reference_frame)].actInv(data.com[0]) -
            target;
    }

    void compute_jacobian(const model_t &model, data_t &data,
                          matrix_ref_t jac) override {
        jac = data.oMf[model.getFrameId(reference_frame)]
                  .toActionMatrixInverse()
                  .topLeftCorner(3, 3) *
              data.Jcom;
    }

    // Target point for the Centre of Mass in the task's reference frame
    vector3_t target;

   private:
    string_t reference_frame;
};

}  // namespace ik