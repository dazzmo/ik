#pragma once

#include <Eigen/Core>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ik {

using Integer = int;
using Index = Eigen::Index;
using Real = double;
using String = std::string;

using Model = pinocchio::ModelTpl<Real>;
using Data = pinocchio::DataTpl<Real>;

}  // namespace ik