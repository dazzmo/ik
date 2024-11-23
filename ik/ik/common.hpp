#pragma once

#include <Eigen/Core>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ik {

typedef std::string string_t;

typedef pinocchio::Model model_t;
typedef pinocchio::Data data_t;

typedef pinocchio::SE3 se3_t;

template <typename T, int Size = Eigen::Dynamic>
using eigen_vector_tpl_t = Eigen::Vector<T, Size>;

template <typename T, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
using eigen_matrix_tpl_t = Eigen::Matrix<T, Rows, Cols>;

typedef eigen_vector_tpl_t<double> eigen_vector_t;
typedef eigen_vector_tpl_t<double, 3> eigen_vector3_t;

typedef eigen_matrix_tpl_t<double> eigen_matrix_t;
typedef eigen_matrix_tpl_t<double, 3, 3> eigen_matrix3_t;

template <typename Scalar>
struct eigen_to_std_vector {
    static inline std::vector<Scalar> convert(
        const eigen_vector_tpl_t<Scalar> &v) {
        return std::vector<Scalar>(v.data(), v.data() + v.rows() * v.cols());
    }
};

inline const se3_t &get_transform_frame_to_world(
    const model_t &model, const data_t &data,
    const string_t &frame_name) {
    return data.oMf[model.getFrameId(frame_name)];
}

inline void apply_joint_clipping(const model_t &model, eigen_vector_t &q) {
    q.noalias() =
        model.upperPositionLimit.cwiseMin(q.cwiseMax(model.lowerPositionLimit));
}

}  // namespace ik