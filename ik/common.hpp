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

template <typename T, int Size = Eigen::Dynamic>
using eigen_vector_tpl_t = Eigen::Vector<T, Size>;

template <typename T, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
using eigen_matrix_tpl_t = Eigen::Matrix<T, Rows, Cols>;

typedef eigen_vector_tpl_t<double> eigen_vector_t;
typedef eigen_vector_tpl_t<double, 3> eigen_vector3_t;

typedef eigen_matrix_tpl_t<double> eigen_matrix_t;
typedef eigen_matrix_tpl_t<double, 3, 3> eigen_matrix3_t;

/**
 * @brief State of a given system
 *
 * @tparam Vector
 */
template <typename VectorType>
struct state {
    state(const std::size_t &nq, const std::size_t &nv)
        : position(nq), velocity(nv), acceleration(nv) {}

    VectorType position;
    VectorType velocity;
    VectorType acceleration;
};

}  // namespace ik