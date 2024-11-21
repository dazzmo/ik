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
#include "ik/ik.hpp"
#include "ik/visitor.hpp"

namespace ik {

struct pik_parameters {
    int max_iterations = 100;
    double damping = 1e-2;
    double step_length = 1.0;
    double max_time = 1.0;
};

struct joint_clipping {
    operator()(const model_t & model, eigen_vector_t & q) {
        q.noalias() = model.upperPositionLimit.cwiseMin(
            q.cwiseMax(model.lowerPositionLimit));
    }
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
         const dls_parameters &p = dls_parameters()) {
    // Parameters we can decide on later
    std::size_t sz = ik.e_size();

    // Initialise jacobian
    eigen_vector_t q = q0, dq(model.nv), e(sz);
    eigen_matrix_t J(sz, model.nv), JJ(sz, sz);

    data_t data = pinocchio::Data(ik.data);

    // todo - if limited convergence, try random walk

    // Perform iterations
    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Update model
        pinocchio::framesForwardKinematics(ik.model, data, q);

        eigen_matrix_t P, J;
        // Compute priority levels
        for (std::size_t priority = 0; i < 10; ++i) {
            // Evaluate error and jacobian
            for (auto &task : ik.get_position_tasks()) {
                if (task.second->priority == priority) {
                    e.middleRows(cnt, 3) =
                        get_task_error(ik.model, *task.second).position;
                    J.middleRows(cnt, 3) = task.second->get_task_jacobian();
                    cnt += 3;
                }
            }

            for (auto &task : ik.get_orientation_tasks()) {
                e.middleRows(cnt, 3) =
                    get_task_error(ik.model, *task.second).position;
                J.middleRows(cnt, 3) = task.second->get_task_jacobian();
                cnt += 3;
            }

            for (auto &task : ik.get_se3_tasks()) {
                e.middleRows(cnt, 6) =
                    get_task_error(ik.model, *task.second).twist.asVector();
                J.middleRows(cnt, 6) = task.second->get_task_jacobian();
                cnt += 6;
            }

            J = J * P;
            dq += J * dq;
        }


        // Other tasks ...

        // dq = dq + P * ;

        // Take a step
        q = pinocchio::integrate(model, q, p.step_length * dq);

        if (visitor.should_stop(ik, dq)) {
            return;
        }

        joint_clipping::operator()(ik.model, q);
    }
}

}  // namespace ik