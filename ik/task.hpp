#pragma once

#include <bopt/program.hpp>

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

#include <Eigen/Core>
#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ik/common.hpp"

namespace ik {

template <class T>
struct task_traits {
    typedef typename T::value_type value_type;
    typedef typename T::index_type index_type;
    typedef typename T::integer_type integer_type;

    typedef typename T::task_state_t task_state_t;
    typedef typename T::task_error_t task_error_t;
    typedef typename T::reference_t reference_t;
};

struct task_attributes {};

template <typename Scalar>
struct eigen_to_std_vector {
    static inline std::vector<Scalar> convert(
        const eigen_vector_tpl_t<Scalar> &v) {
        return std::vector<Scalar>(v.data(), v.data() + v.rows() * v.cols());
    }
};

template <typename T, typename I>
class Task {
   public:
    typedef T value_type;
    typedef I index_type;
    typedef int integer_type;

    Task(const index_type &dimension, const model_t &model)
        : dimension_(dimension), model_nq_(model.nq), model_nv_(model.nv) {}

    index_type priority() { return 0; }

    index_type dimension() const { return dimension_; }
    index_type model_nq() const { return model_nq_; }
    index_type model_nv() const { return model_nv_; }

   private:
    // Dimension of the task
    index_type dimension_;
    index_type model_nq_;
    index_type model_nv_;
};

/**
 * @brief Position task
 *
 */
class PositionTask : public Task<double, std::size_t> {
   public:
    PositionTask(const model_t &model, const std::string &frame_name,
                 const std::string &reference_frame)
        : Task<double, std::size_t>(3, model),
          target_frame(frame_name),
          reference_frame(reference_frame) {
        frame_jacobian_ = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    }

    struct task_state {
        eigen_vector3_t position = eigen_vector3_t::Zero();
    };

    typedef task_state task_state_t;
    typedef task_state task_error_t;
    typedef task_state task_tolerance_t;
    typedef task_state_t reference_t;

    integer_type get_task_state(const model_t &model, const data_t &data,
                                task_state_t &task_state) const {
        task_state.position =
            data.oMf[model.getFrameId(reference_frame)]
                .actInv(data.oMf[model.getFrameId(target_frame)])
                .translation();
        return integer_type(0);
    }

    integer_type get_task_jacobian(const model_t &model, data_t &data,
                                   Eigen::Ref<eigen_matrix_t> jacobian) {
        pinocchio::getFrameJacobian(model, data, model.getFrameId(target_frame),
                                    pinocchio::WORLD, frame_jacobian_);
        VLOG(10) << "get_task_jacobian: J " << frame_jacobian_;
        jacobian = (data.oMf[model.getFrameId(reference_frame)]
                        .toActionMatrixInverse() *
                    frame_jacobian_)
                       .topRows(3);

        return integer_type(0);
    }

    integer_type get_task_error(const task_state_t &task_state,
                                task_error_t &error) const {
        error.position = task_state.position - reference.position;
        return integer_type(0);
    }

    reference_t reference;
    task_tolerance_t tolerance;

    string_t target_frame;
    string_t reference_frame;

   private:
    pinocchio::Data::Matrix6x frame_jacobian_;
};

// class OrientationTask : public Task<double, std::size_t> {
//    public:
//     OrientationTask(const model_t &model, const std::string &frame_name,
//                     const std::string &reference_frame);

//     struct task_state {
//         eigen_matrix3_t rotation = eigen_matrix3_t::Identity();
//     };

//     struct error_state {
//         eigen_vector3_t rotation = eigen_vector3_t::Zero();
//     };

//     typedef task_state task_state_t;
//     typedef error_state task_error_t;
//     typedef error_state task_tolerance_t;
//     typedef task_state_t reference_t;

//     integer_type get_task_state(const model_t &model, const data_t &data,
//                                 task_state_t &task_state) const {
//         task_state.rotation =
//             data.oMf[model.getFrameId(reference_frame)]
//                 .actInv(data.oMf[model.getFrameId(target_frame)])
//                 .rotation();
//         return integer_type(0);
//     }

//     integer_type get_task_jacobian(const model_t &model, data_t &data,
//                                    eigen_matrix_t &jacobian) {
//         pinocchio::getFrameJacobian(model, data,
//         model.getFrameId(target_frame),
//                                     pinocchio::WORLD, jacobian);
//         Eigen::Matrix<double, 3, 3> Jlog;
//         pinocchio::Jlog3(reference.rotation.transpose() *
//         task_state.rotation,
//                          Jlog);
//         jacobian = data.oMf[model.getFrameId(reference_frame)]
//                        .actInv(jacobian)
//                        .bottomRows(3);
//     }

//     integer_type get_task_error(const task_state_t &task_state,
//                                 task_error_t &error) const {
//         error.rotation = pinocchio::log3(reference.rotation.transpose() *
//                                          task_state.rotation);

//         return integer_type(0);
//     }

//     reference_t reference;
//     string_t target_frame;
//     string_t reference_frame;

//    private:
// };

// class CentreOfMassTask : public Task {
//    public:
//     CentreOfMassTask(const model_t &model,
//                      const std::string &reference_frame);

//     struct task_state {
//         eigen_vector3_t position = eigen_vector3_t::Zero();
//         eigen_vector3_t velocity = eigen_vector3_t::Zero();
//     };

//     typedef task_state task_state_t;
//     typedef task_state_t reference_t;

//     integer_type get_task_state(const model_state_t &model_state,
//                                 task_state_t &task_state) const {
//         return integer_type(0);
//     }

//     integer_type get_task_error(const task_state_t &task_state,
//                                 task_error_t &error,
//                                 const value_type &dt = 0.0) const {
//         return integer_type(0);
//     }

//     reference_t reference;
//     string_t target_frame;
//     string_t reference_frame;

//    private:
//     typedef bopt::casadi::expression_evaluator<value_type>
//         expression_evaluator_t;
//     std::unique_ptr<expression_evaluator_t> xpos;
//     std::unique_ptr<expression_evaluator_t> xvel;
// };

// class SE3Task : public Task<double, std::size_t> {
//    public:
//     SE3Task(const model_t &model, const std::string &target_frame,
//             const std::string &reference_frame);

//     typedef pinocchio::SE3 se3_t;
//     typedef pinocchio::Motion twist_t;

//     struct task_state {
//         se3_t pose;
//     };

//     struct error_state {
//         twist_t twist;
//     };

//     typedef task_state task_state_t;
//     typedef error_state error_state_t;
//     typedef task_state_t reference_t;

//     integer_type get_task_state(const model_t &model, data_t &data,
//                                 task_state_t &task_state) const {
//         task_state.pose = data.oMf[model.getFrameId(reference_frame)].actInv(
//             data.oMf[model.getFrameId(target_frame)]);
//         return integer_type(0);
//     }

//     integer_type get_task_error(const task_state_t &task_state,
//                                 task_error_t &error,
//                                 const value_type &dt = 0.0) const {
//         Eigen::Matrix<double, 6, 6> Jlog;
//         error.positional =
//             pinocchio::log6(reference.pose.actInv(task_state.pose)).toVector();
//         pinocchio::Jlog6(reference.pose.actInv(task_state.pose), Jlog);
//         return integer_type(0);
//     }

//     reference_t reference;
//     string_t target_frame;
//     string_t reference_frame;

//    private:
// };

}  // namespace ik