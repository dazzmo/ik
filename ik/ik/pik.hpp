/**
 * @file pik.hpp
 * @author your name (you@domain.com)
 * @brief Priority based inverse kinematics approach
 * @version 0.1
 * @date 2024-11-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "ik/program.hpp"
#include "ik/visitor.hpp"

namespace ik {

struct pik_parameters {
    int max_iterations = 100;
    double damping = 1e-2;
    double step_length = 1.0;
    double max_time = 1.0;
};

class FrameTask : public FrameTask {
   public:
    std::size_t priority = 0;
};

// todo - priority version

/**
 * @brief Priority based inverse kinematics
 *
 * @param ik
 * @param q0
 * @param visitor
 * @param p
 */
void pik(ik &ik, const eigen_vector_t &q0,
         inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
         const pik_parameters &p = pik_parameters()) {
    // Parameters we can decide on later
    std::size_t sz = ik.e_size();

    // Initialise jacobian
    eigen_vector_t q = q0, dq(model.nv), e(sz);
    eigen_matrix_t J(sz, model.nv), JJ(sz, sz);

    data_t data = pinocchio::Data(ik.data);

    // todo - if limited convergence, try random walk

    // Organise tasks into their priorities
    std::vector<std::vector<std::shared_ptr<Task<double, std::size_t>>>> tasks;

    for (std::size_t priority = 0; priority < 10; ++priority) {
        // Evaluate error and jacobian
        for (auto &task : ik.get_position_task()) {
            if (task->priority == priority) {
                tasks_[priority].push_back(task);
            }
        }
    }

    // Perform iterations
    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Update model
        pinocchio::framesForwardKinematics(ik.model, data, q);

        eigen_matrix_t P, J;
        // Compute priority levels
        for (std::size_t priority = 0; i < 10; ++i) {
            // Evaluate error and jacobian
            eigen_matrix_t Ji, ei;

            J = J * P;
            dq += J * dq;
        }

        // Other tasks ...

        // dq = dq + P * ;
    }
}

}  // namespace ik