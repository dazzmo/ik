#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/constraint.hpp"
#include "ik/task.hpp"

namespace ik {

#define FRAME_POSITION_DIM 3
#define FRAME_ORIENTATION_DIM 3
#define FRAME_AXIS_ALIGN_DIM 3
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
 * @class FrameTask
 * @brief A template class for defining inverse kinematics tasks for specific
 * frames in a robotic system.
 *
 * This class inherits from `Task` and provides the functionality to define
 * tasks associated with specific frames of a robot (e.g., end-effector
 * positions or orientations) relative to a reference frame. It supports three
 * types of tasks: position-only, orientation-only, and full 6D pose (position +
 * orientation).
 *
 * @tparam ValueType The scalar type used for computations (e.g., `double` or
 * `float`).
 */
class FrameTask : public Task {
   public:
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     */
    FrameTask(const model_t &model, const std::string &frame,
              const KinematicType &type = KinematicType::Full,
              const std::string &reference_frame = "universe")
        : Task(),
          target(se3_t::Identity()),
          type(type),
          frame(frame),
          reference_frame(reference_frame) {
        // assert(model.getFrameId(frame) < model.frames().size() &&
        //        "Frame not found in model");
        // assert(model.getFrameId(reference_frame) < model.frames().size() &&
        //    "Reference frame not found in model");
        // Set task dimension based on task type
        if (type == KinematicType::Position) {
            this->set_dimension(index_t(FRAME_POSITION_DIM));
        } else if (type == KinematicType::Orientation) {
            this->set_dimension(index_t(FRAME_ORIENTATION_DIM));
        } else {
            this->set_dimension(index_t(FRAME_POSE_DIM));
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
     * @return A shared pointer to the created `FrameTask` instance.
     */
    static std::shared_ptr<FrameTask> create(
        const model_t &model, const std::string &frame,
        const KinematicType &type = KinematicType::Full,
        const std::string &reference_frame = "universe") {
        return std::make_shared<FrameTask>(model, frame, type, reference_frame);
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
        data_t::Matrix6 Jlog;
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
    // Frame name
    string_t frame;
    // Reference frame name
    string_t reference_frame;
    // Matrix to compute the frame jacobians of the task
    pinocchio::Data::Matrix6x frame_jacobian_;
};

enum class AlignAxisType { AxisX = 0, AxisY = 1, AxisZ = 2 };

/**
 * @brief Task designed to align a particular axis of an end-effector frame to,
 * irrespective of the other axes of the frame. Appropriate for contact tasks
 * where having the end-effector aligned with the contact normal is essential.
 *
 */
class AlignAxisTask : public Task {
   public:
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     */
    AlignAxisTask(const model_t &model, const std::string &frame,
                  const AlignAxisType &axis,
                  const std::string &reference_frame = "universe")
        : Task(), axis_(axis), frame(frame), reference_frame(reference_frame) {
        // Set dimension
        this->set_dimension(index_t(1));
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
     * @return A shared pointer to the created `FrameTask` instance.
     */
    static std::shared_ptr<AlignAxisTask> create(
        const model_t &model, const std::string &frame,
        const AlignAxisType &axis,
        const std::string &reference_frame = "universe") {
        return std::make_shared<AlignAxisTask>(model, frame, axis,
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
        const auto &oMf = get_transform_frame_to_world(model, data, frame);
        // Reference Frame to World
        const auto &oMr =
            get_transform_frame_to_world(model, data, reference_frame);
        // Frame to Reference Frame
        auto rMf = oMr.actInv(oMf);
        // Get axis of frame with respect to the reference frame
        Eigen::Ref<const vector3_t> r =
            rMf.rotation().col(static_cast<Eigen::Index>(axis_));

        // Compute alignment error
        e << 1.0 - r.dot(target.normalized());
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
        // Compute the frame error
        const auto &oMf = get_transform_frame_to_world(model, data, frame);
        // Reference Frame to World
        const auto &oMr =
            get_transform_frame_to_world(model, data, reference_frame);
        // Frame to Reference Frame
        auto rMf = oMr.actInv(oMf);

        // Compute Jacobian of end-effector in local frame
        pinocchio::getFrameJacobian(model, data, model.getFrameId(frame),
                                    pinocchio::LOCAL, frame_jacobian_);

        // Create task Jacobian
        jac = -(rMf.rotation()
                    .col(static_cast<Eigen::Index>(axis_))
                    .cross(target.normalized()))
                   .transpose() *
              rMf.rotation() * frame_jacobian_.bottomRows(3);
    }

    /**
     * @brief Desired axis to align the specified frame axis to
     *
     */
    vector3_t target;

   protected:
    // Axis of the frame we align
    AlignAxisType axis_;

    // Frame name
    string_t frame;
    // Reference frame name
    string_t reference_frame;
    // Matrix to compute the frame jacobians of the task
    data_t::Matrix6x frame_jacobian_;
};

/**
 * @class FrameConstraint
 * @brief A template class for defining an inverse kinematics constraint for a
 * frame, such that the frame is fixed relative to a given reference frame.
 *
 * This class inherits from `Constraint` and provides the functionality to
 * define constraints associated with specific frames of a robot (e.g.,
 * end-effector positions or orientations) relative to a reference frame. It
 * supports three types of constraints: position-only, orientation-only, and
 * full 6D pose (position + orientation).
 *
 */
class FrameConstraint : public Constraint {
   public:
    /**
     * @brief Constructor for creating a frame task.
     *
     * @param model The Pinocchio model of the robot.
     * @param frame The name of the frame for which the task is defined.
     * @param type The type of task (default: `KinematicType::Full`).
     * @param reference_frame The name of the reference frame for the task
     * (default: "universe").
     */
    FrameConstraint(const model_t &model, const std::string &frame,
                    const KinematicType &type = KinematicType::Full,
                    const std::string &reference_frame = "universe")
        : Constraint(),
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
            this->set_dimension(index_t(FRAME_POSITION_DIM));
        } else if (type == KinematicType::Orientation) {
            this->set_dimension(index_t(FRAME_ORIENTATION_DIM));
        } else {
            this->set_dimension(index_t(FRAME_POSE_DIM));
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
     * @return A shared pointer to the created `FrameConstraint` instance.
     */
    static std::shared_ptr<FrameConstraint> create(
        const model_t &model, const std::string &frame,
        const KinematicType &type = KinematicType::Full,
        const std::string &reference_frame = "universe") {
        return std::make_shared<FrameConstraint>(model, frame, type,
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
    data_t::Matrix6x frame_jacobian_;
    data_t::Matrix6x reference_frame_jacobian_;

    // Frame name
    string_t frame;
    // Reference frame name
    string_t reference_frame;
};

}  // namespace ik