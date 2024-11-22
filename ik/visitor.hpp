#include "ik/common.hpp"
#include "ik/ik.hpp"

namespace ik {

class inverse_kinematics_visitor {
   public:
    inverse_kinematics_visitor() = default;
    bool update_error() const { return true; }
    bool update_jacobian() const { return true; }
    bool should_stop(const ik& ik, const eigen_vector_t& dq) const {
        // Assess if all errors are within a tolerance
        return false;
    }
};

}  // namespace ik