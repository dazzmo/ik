#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/constraint.hpp"
#include "ik/task.hpp"

namespace ik {

#define FRAME_POSITION_DIM 3
#define FRAME_ORIENTATION_DIM 3
#define FRAME_POSE_DIM 6

/**
 * @brief Indication of which degrees of freedom are being constrained or
 * controlled.
 *
 */
enum class KinematicType { Position, Orientation, Full };

/**
 * @brief Computes the error between the frame `frame` on the kinematic model
 * and a target frame `target`, expressed in the reference frame
 * `reference_frame`. Returns the logarithm of the difference in  between the
 * two frames for the current configuration of the system in `e`, provided by
 * `model` and `data`.
 *
 * @param model
 * @param data
 * @param e
 * @param frame
 * @param target
 * @param reference_frame
 * @param type
 */
inline void compute_frame_error(
    const model_t &model, data_t &data, vector_ref_t e,
    const std::string &frame, const pinocchio::SE3 &target,
    const std::string &reference_frame,
    const KinematicType &type = KinematicType::Full) {
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
    if (type == KinematicType::Position) {
        e = pinocchio::log6(fMt).linear();
    }
    if (type == KinematicType::Orientation) {
        e = pinocchio::log6(fMt).angular();
    }
    if (type == KinematicType::Full) {
        e = pinocchio::log6(fMt).toVector();
    }
}

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

    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     */
    FrameTaskTpl(const model_t &model, const std::string &frame,
                 const KinematicType &type = KinematicType::Full,
                 const std::string &reference_frame = "universe")
        : TaskTpl<ValueType, IndexType, IntegerType>(),
          frame(frame),
          type(type),
          reference_frame(reference_frame),
          target(se3_t::Identity()) {
        // assert(model.getFrameId(frame) < model.frames().size() &&
        //        "Frame not found in model");
        // assert(model.getFrameId(reference_frame) < model.frames().size() &&
        //    "Reference frame not found in model");
        // Set task dimension based on task type
        if (type == KinematicType::Position) {
            this->set_dimension(index_type(FRAME_POSITION_DIM));
        } else if (type == KinematicType::Orientation) {
            this->set_dimension(index_type(FRAME_ORIENTATION_DIM));
        } else {
            this->set_dimension(index_type(FRAME_POSE_DIM));
        }
        // Initialise frame jacobian matrix
        frame_jacobian_ = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    }

    /**
     * @brief Factory method to create a shared pointer to a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     * @return A shared pointer to the created `FrameTaskTpl` instance.
     */
    static std::shared_ptr<FrameTaskTpl> create(
        const model_t &model, const std::string &frame,
        const KinematicType &type = KinematicType::Full,
        const std::string &reference_frame = "universe") {
        return std::make_shared<FrameTaskTpl>(model, frame, type,
                                              reference_frame);
    }

    /**
     * @brief Computes the tasj error between the current and target frame
     * configurations.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param e The vector to store the computed error.
     */
    void compute_error(const model_t &model, data_t &data,
                       const vector_const_ref_t q, vector_ref_t e) override {
        // Compute the frame error
        compute_frame_error(model, data, e, frame, target, reference_frame,
                            type);
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
        if (type == KinematicType::Position) {
            jac = (-Jlog * frame_jacobian_).topRows(3);
        }
        if (type == KinematicType::Orientation) {
            jac = (-Jlog * frame_jacobian_).bottomRows(3);
        }
        if (type == KinematicType::Full) {
            jac = (-Jlog * frame_jacobian_);
        }
    }

    /**
     * @brief Target frame for the task, with respect to the specified reference
     * frame for the task
     *
     */
    se3_t target;

   protected:
    // Kinematic type of the task
    KinematicType type;

    // Matrix to compute the frame jacobians of the task
    pinocchio::Data::Matrix6x frame_jacobian_;

    // Frame name
    string_t frame;
    // Reference frame name
    string_t reference_frame;
};

typedef FrameTaskTpl<double> FrameTask;

/**
 * @class FrameConstraintTpl
 * @brief A template class for defining an inverse kinematics constraint for a
 * frame, such that the frame is fixed relative to a given reference frame.
 *
 * This class inherits from `ConstraintTpl` and provides the functionality to
 * define constraints associated with specific frames of a robot (e.g.,
 * end-effector positions or orientations) relative to a reference frame. It
 * supports three types of constraints: position-only, orientation-only, and
 * full 6D pose (position + orientation).
 *
 * @tparam ValueType The scalar type used for computations (e.g., `double` or
 * `float`).
 * @tparam IndexType The type used for indexing (default: `std::size_t`).
 * @tparam IntegerType The type used for integer values (default: `int`).
 */
template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class FrameConstraintTpl
    : public ConstraintTpl<ValueType, IndexType, IntegerType> {
   public:
    typedef pinocchio::SE3 se3_t;
    typedef pinocchio::Motion twist_t;

    typedef ConstraintTpl<ValueType, IndexType, IntegerType> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     */
    FrameConstraintTpl(const model_t &model, const std::string &frame,
                       const KinematicType &type = KinematicType::Full,
                       const std::string &reference_frame = "universe")
        : ConstraintTpl<ValueType, IndexType, IntegerType>(),
          frame(frame),
          type(type),
          reference_frame(reference_frame),
          target(se3_t::Identity()) {
        // assert(model.getFrameId(frame) < model.frames().size() &&
        //        "Frame not found in model");
        // assert(model.getFrameId(reference_frame) < model.frames().size() &&
        //    "Reference frame not found in model");
        // Set task dimension based on task type
        if (type == KinematicType::Position) {
            this->set_dimension(index_type(FRAME_POSITION_DIM));
        } else if (type == KinematicType::Orientation) {
            this->set_dimension(index_type(FRAME_ORIENTATION_DIM));
        } else {
            this->set_dimension(index_type(FRAME_POSE_DIM));
        }
        // Initialise frame jacobian matrix
        frame_jacobian_ = pinocchio::Data::Matrix6x::Zero(6, model.nv);
        // Initialise frame jacobian matrix
        reference_frame_jacobian_ =
            pinocchio::Data::Matrix6x::Zero(6, model.nv);
    }

    /**
     * @brief Factory method to create a shared pointer to a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     * @return A shared pointer to the created `FrameConstraintTpl` instance.
     */
    static std::shared_ptr<FrameConstraintTpl> create(
        const model_t &model, const std::string &frame,
        const KinematicType &type = KinematicType::Full,
        const std::string &reference_frame = "universe") {
        return std::make_shared<FrameConstraintTpl>(model, frame, type,
                                                    reference_frame);
    }

    /**
     * @brief Computes the constraint error between the current and target frame
     * configurations.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param e The vector to store the computed error.
     */
    void compute_error(const model_t &model, data_t &data,
                       const vector_const_ref_t q, vector_ref_t e) override {
        // Compute the frame error
        compute_frame_error(model, data, e, frame, target, reference_frame,
                            type);
    }

    /**
     * @brief Computes the constraint Jacobian matrix, by computing the relative
     * velocity between the target frame and the reference frame, expressed in
     * the local frame of the end-effector.
     *
     * @param model The Pinocchio model of the robot.
     * @param data The Pinocchio data structure for the robot.
     * @param jac The matrix to store the computed Jacobian.
     */
    void compute_jacobian(const model_t &model, data_t &data,
                          matrix_ref_t jac) override {
        // Frame to World
        const auto &oMf =
            get_transform_frame_to_world(model, data, this->frame);
        // Reference Frame to World
        const auto &oMr =
            get_transform_frame_to_world(model, data, this->reference_frame);
        // Frame to Reference Frame
        auto rMf = oMr.actInv(oMf);

        // Compute Jacobian of end-effector frame in local frame
        pinocchio::getFrameJacobian(model, data, model.getFrameId(this->frame),
                                    pinocchio::LOCAL, this->frame_jacobian_);

        // Compute Jacobian of the reference frame in its local frame
        pinocchio::getFrameJacobian(
            model, data, model.getFrameId(this->reference_frame),
            pinocchio::LOCAL, this->reference_frame_jacobian_);

        // Compute constraint jacobian of relative velocity in the local frame
        // of the target frame
        if (this->type == KinematicType::Position) {
            jac = (this->frame_jacobian_ - rMf.toActionMatrixInverse() *
                                               this->reference_frame_jacobian_)
                      .topRows(3);
        }
        if (this->type == KinematicType::Orientation) {
            jac = (this->frame_jacobian_ - rMf.toActionMatrixInverse() *
                                               this->reference_frame_jacobian_)
                      .bottomRows(3);
        }
        if (this->type == KinematicType::Full) {
            jac = (this->frame_jacobian_ - rMf.toActionMatrixInverse() *
                                               this->reference_frame_jacobian_);
        }
    }

    se3_t target;

   protected:
    // Kinematic type of the constraint
    KinematicType type;

    // Matrix to compute the frame jacobians of the task
    pinocchio::Data::Matrix6x frame_jacobian_;
    pinocchio::Data::Matrix6x reference_frame_jacobian_;

    // Frame name
    string_t frame;
    // Reference frame name
    string_t reference_frame;
};

typedef FrameConstraintTpl<double> FrameConstraint;

}  // namespace ik