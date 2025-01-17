#pragma once

#include <Eigen/Core>
// #include <casadi/casadi.hpp>
// #include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ik {

typedef int int_t;
typedef std::size_t index_t;
typedef double number_t;

typedef std::string string_t;

typedef pinocchio::ModelTpl<number_t> model_t;
typedef pinocchio::DataTpl<number_t> data_t;

typedef pinocchio::SE3Tpl<number_t> se3_t;

template <typename T, int Size = Eigen::Dynamic>
using vector_tpl_t = Eigen::Vector<T, Size>;

template <typename T, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
using matrix_tpl_t = Eigen::Matrix<T, Rows, Cols>;

typedef vector_tpl_t<double> vector_t;
typedef vector_tpl_t<double, 3> vector3_t;

typedef Eigen::Ref<vector_t> vector_ref_t;
typedef Eigen::Ref<const vector_t> vector_const_ref_t;

typedef matrix_tpl_t<double> matrix_t;
typedef matrix_tpl_t<double, 3, 3> matrix3_t;

typedef Eigen::Ref<matrix_t> matrix_ref_t;
typedef Eigen::Ref<const matrix_t> matrix_const_ref_t;

template <typename Scalar>
struct eigen_to_std_vector {
    static inline std::vector<Scalar> convert(const vector_tpl_t<Scalar> &v) {
        return std::vector<Scalar>(v.data(), v.data() + v.rows() * v.cols());
    }
};

inline const se3_t &get_transform_frame_to_world(const model_t &model,
                                                 const data_t &data,
                                                 const string_t &frame_name) {
    return data.oMf[model.getFrameId(frame_name)];
}

inline void apply_joint_clipping(const model_t &model, vector_t &q) {
    q.noalias() =
        model.upperPositionLimit.cwiseMin(q.cwiseMax(model.lowerPositionLimit));
}

// Default solver parameters
struct default_solver_parameters {
    // Maximum number of iterations to perform
    int max_iterations = 100;
    // Maximum solve time allowable
    double max_time = 1.0;
    // Step length to perform
    double step_length = 1.0;
};

}  // namespace ik