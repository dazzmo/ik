/**
 * @file dls.hpp
 * @author your name (you@domain.com)
 * @brief Damped-Least Squares approach
 * @version 0.1
 * @date 2024-11-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "ik/ik.hpp"
#include "ik/visitor.hpp"

namespace ik {

struct dls_parameters {
    int max_iterations = 100;
    double damping = 1e-2;
    double step_length = 1.0;
    double max_time = 1.0;
};

void apply_joint_clipping(const model_t &model, eigen_vector_t &q) {
    q.noalias() =
        model.upperPositionLimit.cwiseMin(q.cwiseMax(model.lowerPositionLimit));
}

// todo - priority version

void dls(
    ik &ik, const eigen_vector_t &q0,
    const inverse_kinematics_visitor &visitor = inverse_kinematics_visitor(),
    const dls_parameters &p = dls_parameters()) {
    // Parameters we can decide on later
    std::size_t sz = ik.e_size();

    // Initialise jacobian
    eigen_vector_t q = q0, dq(ik.model.nv), e(sz);
    eigen_matrix_t J(sz, ik.model.nv), JJ(sz, sz);

    data_t data = pinocchio::Data(ik.model);

    // todo - if limited convergence, try random walk

    // Perform iterations
    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Update model
        pinocchio::framesForwardKinematics(ik.model, data, q);
        pinocchio::computeJointJacobians(ik.model, data);
        
        std::size_t cnt = 0;
        // Evaluate error and jacobian
        for (auto &task : ik.get_position_tasks()) {
            e.middleRows(cnt, 3) =
                get_task_error(ik.model, data, *task.second).position;
            task.second->get_task_jacobian(ik.model, data,
                                           J.middleRows(cnt, 3));
            cnt += 3;
        }

        // for (auto &task : ik.get_orientation_tasks()) {
        //     e.middleRows(cnt, 3) =
        //         get_task_error(ik.model, *task.second).position;
        //     J.middleRows(cnt, 3) = task.second->get_task_jacobian();
        //     cnt += 3;
        // }

        // for (auto &task : ik.get_se3_tasks()) {
        //     e.middleRows(cnt, 6) =
        //         get_task_error(ik.model, *task.second).twist.asVector();
        //     J.middleRows(cnt, 6) = task.second->get_task_jacobian();
        //     cnt += 6;
        // }

        // Other tasks ...

        // Estimate hessian of cost
        JJ.noalias() = J.transpose() * J;
        JJ.diagonal().array() += p.damping * p.damping;
        // Compute step direction
        dq = -JJ.ldlt().solve(J.transpose() * e);
        
        VLOG(10) << "dls: it = " << i;
        VLOG(10) << "dls: J = " << J;
        VLOG(10) << "dls: q = " << q.transpose();
        VLOG(10) << "dls: dq = " << dq.transpose();
        VLOG(10) << "dls: e = " << e.transpose();

        // Take a step
        q = pinocchio::integrate(ik.model, q, p.step_length * dq);

        if (visitor.should_stop(ik, dq)) {
            return;
        }

        apply_joint_clipping(ik.model, q);

        // If issues, perform random restart
    }

    // If it didn't converge, try a random restart?
}

}  // namespace ik