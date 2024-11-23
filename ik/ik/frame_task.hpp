#pragma once

#include <pinocchio/algorithm/frames.hpp>

#include "ik/task.hpp"

namespace ik {

/**
 * @brief Position task
 *
 */
class FrameTask : public Task {
   public:
    typedef pinocchio::SE3 se3_t;
    typedef pinocchio::Motion twist_t;

    enum class Type { Position, Orientation, Full };

    FrameTask(const model_t &model, const std::string &frame,
              const Type &type = Type::Full,
              const std::string &reference_frame = "universe")
        : Task(), frame(frame), type(type), reference_frame(reference_frame) {
        if (type == Type::Position || type == Type::Orientation) {
            set_dimension(index_type(3));
        } else {
            set_dimension(index_type(6));
        }
        frame_jacobian_ = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    }

    void compute_error(const model_t &model, data_t &data,
                       error_vector_ref_type e) override {
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

    void compute_jacobian(const model_t &model, data_t &data,
                          jacobian_matrix_ref_type jac) override {
        // Frame to World
        const auto &oMf = get_transform_frame_to_world(model, data, frame);
        // Reference Frame to World
        const auto &oMr =
            get_transform_frame_to_world(model, data, reference_frame);
        // Target to World
        auto oMt = oMr.act(target);
        // Frame to Target
        auto tMf = oMt.actInv(oMf);

        Eigen::Matrix<double, 6, 6> Jlog;
        pinocchio::Jlog6(tMf, Jlog);

        // Compute Jacobian of end-effector in local frame
        pinocchio::getFrameJacobian(model, data, model.getFrameId(frame),
                                    pinocchio::LOCAL, frame_jacobian_);

        // Create weighted task Jacobian
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

    se3_t target;

   private:
    Type type;

    pinocchio::Data::Matrix6x frame_jacobian_;

    string_t frame;
    string_t reference_frame;
};

class CentreOfMassTask : public Task {
   public:
    CentreOfMassTask(const std::string &reference_frame)
        : Task(3), reference_frame(reference_frame) {};

    void compute_error(const model_t &model, data_t &data,
                       Eigen::Ref<eigen_vector_t> e) override {
        e = data.oMf[model.getFrameId(reference_frame)].actInv(data.com[0]) -
            target;
    }

    void compute_jacobian(const model_t &model, data_t &data,
                          Eigen::Ref<eigen_matrix_t> jac) override {
        jac = data.oMf[model.getFrameId(reference_frame)]
                  .toActionMatrixInverse()
                  .topLeftCorner(3, 3) *
              data.Jcom;
    }

    eigen_vector3_t target;

    string_t reference_frame;

   private:
};

}  // namespace ik