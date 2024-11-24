#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/task.hpp"

namespace ik {

#define FRAME_TASK_POSITION_DIM 3
#define FRAME_TASK_ORIENTATION_DIM 3
#define FRAME_TASK_DIM 6

/**
 * @class FrameTaskTpl
 * @brief A template class for defining inverse kinematics tasks for specific
 * frames in a robotic system.
 *
 * This class inherits from `TaskTpl` and provides the functionality to define
 * tasks associated with specific frames of a robot (e.g., end-effector
 * positions or orientations) relative to a reference frame. It supports three
 * types of tasks: position-only, orientation-only, and full 6D pose (position +
 * orientation).
 *
 * @tparam ValueType The scalar type used for computations (e.g., `double` or
 * `float`).
 * @tparam IndexType The type used for indexing (default: `std::size_t`).
 * @tparam IntegerType The type used for integer values (default: `int`).
 */
template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class FrameTaskTpl : public TaskTpl<ValueType, IndexType, IntegerType> {
   public:
    typedef pinocchio::SE3 se3_t;
    typedef pinocchio::Motion twist_t;

    typedef TaskTpl<ValueType, IndexType, IntegerType> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    enum class Type { Position, Orientation, Full };

    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `Type::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     */
    FrameTaskTpl(const model_t &model, const std::string &frame,
                 const Type &type = Type::Full,
                 const std::string &reference_frame = "universe")
        : TaskTpl<ValueType, IndexType, IntegerType>(),
          frame(frame),
          type(type),
          reference_frame(reference_frame) {
        // assert(model.getFrameId(frame) < model.frames().size() &&
        //        "Frame not found in model");
        // assert(model.getFrameId(reference_frame) < model.frames().size() &&
        //    "Reference frame not found in model");
        // Set task dimension based on task type
        if (type == Type::Position) {
            this->set_dimension(index_type(FRAME_TASK_POSITION_DIM));
        } else if (type == Type::Orientation) {
            this->set_dimension(index_type(FRAME_TASK_ORIENTATION_DIM));
        } else {
            this->set_dimension(index_type(FRAME_TASK_DIM));
        }
        // Initialise frame jacobian matrix
        frame_jacobian_ = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    }

    /**
     * @brief Factory method to create a shared pointer to a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `Type::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     * @return A shared pointer to the created `FrameTaskTpl` instance.
     */
    static std::shared_ptr<FrameTaskTpl> create(
        const model_t &model, const std::string &frame,
        const Type &type = Type::Full,
        const std::string &reference_frame = "universe") {
        return std::make_shared<FrameTaskTpl>(model, frame, type,
                                              reference_frame);
    }

    /**
     * @brief Computes the task error between the current and target frame
     * configurations.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param e The vector to store the computed error.
     */
    void compute_error(const model_t &model, data_t &data,
                       vector_ref_t e) override {
        // Frame to World
        const auto &oMf = get_transform_frame_to_world(model, data, frame);
        // Reference Frame to World
        const auto &oMr =
            get_transform_frame_to_world(model, data, reference_frame);
        // Target to World
        auto oMt = oMr.act(target);
        // Target to Frame
        auto fMt = oMf.actInv(oMt);
        // Compute error between target frame and the current frame of the
        // system
        if (type == Type::Position) {
            e = pinocchio::log6(fMt).linear();
        }
        if (type == Type::Orientation) {
            e = pinocchio::log6(fMt).angular();
        }
        if (type == Type::Full) {
            e = pinocchio::log6(fMt).toVector();
        }
    }

    /**
     * @brief Computes the task Jacobian matrix.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param jac The matrix to store the computed Jacobian.
     */
    void compute_jacobian(const model_t &model, data_t &data,
                          matrix_ref_t jac) override {
        // Frame to World
        const auto &oMf = get_transform_frame_to_world(model, data, frame);
        // Reference Frame to World
        const auto &oMr =
            get_transform_frame_to_world(model, data, reference_frame);
        // Target to World
        auto oMt = oMr.act(target);
        // Frame to Target
        auto tMf = oMt.actInv(oMf);

        // Construct jacobian of the logarithm map
        Eigen::Matrix<double, 6, 6> Jlog;
        pinocchio::Jlog6(tMf, Jlog);

        // Compute Jacobian of end-effector in local frame
        pinocchio::getFrameJacobian(model, data, model.getFrameId(frame),
                                    pinocchio::LOCAL, frame_jacobian_);

        // Create task Jacobian
        if (type == Type::Position) {
            jac = (-Jlog * frame_jacobian_).topRows(3);
        }
        if (type == Type::Orientation) {
            jac = (-Jlog * frame_jacobian_).bottomRows(3);
        }
        if (type == Type::Full) {
            jac = (-Jlog * frame_jacobian_);
        }
    }

    /**
     * @brief Target frame for the task, with respect to the specified reference
     * frame for the task
     *
     */
    se3_t target;

   private:
    // Task type
    Type type;

    // Matrix to compute the frame jacobians of the task
    pinocchio::Data::Matrix6x frame_jacobian_;

    // Frame name
    string_t frame;
    // Reference frame name
    string_t reference_frame;
};

typedef FrameTaskTpl<double> FrameTask;

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
                       vector_ref_t e) override {
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

typedef CentreOfMassTaskTpl<double> CentreOfMassTask;

}  // namespace ik