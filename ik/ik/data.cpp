#include "ik/data.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace ik {

problem_data::problem_data(const InverseKinematicsProblem &problem) {
    q = vector_t::Zero(problem.model().nq);
    dq = vector_t::Zero(problem.model().nv);

    e.resize(problem.max_priority_level() + 1);
    J.resize(problem.max_priority_level() + 1);

    for (std::size_t priority = 0; priority <= problem.max_priority_level();
         ++priority) {
        std::size_t sz = problem.e_size(priority);
        e[priority] = vector_t::Zero(sz);
        J[priority] = matrix_t::Zero(sz, problem.model().nv);
    }

    model_data = pinocchio::Data(problem.model());
}

void evaluate_problem_data(InverseKinematicsProblem &problem,
                           problem_data &data) {
    // Update model
    pinocchio::framesForwardKinematics(problem.model(), data.model_data,
                                       data.q);
    pinocchio::computeJointJacobians(problem.model(), data.model_data);
    if (problem.get_centre_of_mass_task()) {
        pinocchio::jacobianCenterOfMass(problem.model(), data.model_data,
                                        data.q, false);
    }

    std::size_t cnt = 0;
    // Get all priority 0 tasks
    for (std::size_t p = 0; p <= problem.max_priority_level(); ++p) {
        std::size_t idx = 0;
        for (auto &task : problem.get_all_tasks(p)) {
            // Get references
            vector_ref_t ei = data.e[p].middleRows(idx, task->dimension());
            matrix_ref_t Ji = data.J[p].middleRows(idx, task->dimension());

            // Compute task error and jacobians
            task->compute_error(problem.model(), data.model_data, data.q, ei);
            task->compute_jacobian(problem.model(), data.model_data, Ji);
            // Weight tasks
            ei = ei.cwiseProduct(task->weighting());
            Ji = task->weighting().asDiagonal() * Ji;

            // Move to next task
            idx += task->dimension();
        }

        cnt += data.e[p].rows();
    }
}

}  // namespace ik
