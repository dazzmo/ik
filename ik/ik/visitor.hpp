#pragma once
#include "ik/common.hpp"
#include "ik/problem.hpp"

namespace ik {

class inverse_kinematics_visitor {
   public:
    inverse_kinematics_visitor() = default;

    virtual bool update_error() const { return true; }

    virtual bool update_jacobian() const { return true; }

    virtual bool should_stop(const InverseKinematicsProblem& ik,
                             problem_data& data) const {
        // Assess priority 0 tolerances
        if (data.e[0].squaredNorm() < 1e-4) return true;
        return false;
    }
};

class default_inverse_kinematics_visitor : public inverse_kinematics_visitor {};

}  // namespace ik