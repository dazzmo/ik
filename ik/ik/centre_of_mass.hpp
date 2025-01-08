#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/constraint.hpp"
#include "ik/task.hpp"

namespace ik {

/**
 * @brief Task for the centre of mass location (a 3D point in space).
 *
 * @tparam ValueType The scalar type used for computations (e.g., `double` or
 * `float`).
 * @tparam IndexType The type used for indexing (default: `std::size_t`).
 * @tparam IntegerType The type used for integer values (default: `int`).
 */
template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class CentreOfMassTaskTpl : public TaskTpl<ValueType, IndexType, IntegerType> {
   public:
    typedef TaskTpl<ValueType, IndexType, IntegerType> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    CentreOfMassTaskTpl(const model_t &model,
                        const std::string &reference_frame)
        : TaskTpl<ValueType, IndexType, IntegerType>(3),
          reference_frame(reference_frame) {};

    /**
     * @brief Factory method to create a shared pointer to a centre of mass
     * task.
     *
     * @param model The Pinocchio model of the robot.
     * @param reference_frame The name of the reference frame for the centre of
     * mass (default: "universe").
     * @return A shared pointer to the created `CentreOfMassTaskTpl` instance.
     */
    static std::shared_ptr<CentreOfMassTaskTpl> create(
        const model_t &model, const std::string &reference_frame = "universe") {
        return std::make_shared<CentreOfMassTaskTpl>(model, reference_frame);
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

/**
 * @copybrief CentreOfMassTaskTpl()
 *
 */
typedef CentreOfMassTaskTpl<double> CentreOfMassTask;

}  // namespace ik