#include "ik/common.hpp"
#include "ik/problem.hpp"

namespace ik {

class inverse_kinematics_visitor {
   public:
    inverse_kinematics_visitor() = default;
    
    virtual bool update_error() const { return true; }
    
    virtual bool update_jacobian() const { return true; }

    virtual bool should_stop(const InverseKinematicsProblem& ik, const vector_t& e, const vector_t& dq) const {
        if(e.squaredNorm() < 1e-4) return true;
        return false;
    }
};

class default_inverse_kinematics_visitor : public inverse_kinematics_visitor {
};

}  // namespace ik