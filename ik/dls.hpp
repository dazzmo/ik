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
#include <pinocchio/algorithm/center-of-mass.hpp>
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

eigen_vector_t dls(
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

    auto tasks = ik.get_all_tasks();

    // Perform iterations
    for (std::size_t i = 0; i < p.max_iterations; ++i) {
        // Update model
        pinocchio::framesForwardKinematics(ik.model, data, q);
        pinocchio::computeJointJacobians(ik.model, data);
        pinocchio::jacobianCenterOfMass(ik.model, data, q, false);

        std::size_t cnt = 0;
        for (auto &task : tasks) {
            VLOG(10) << "e = " << e;
            task->compute_error(ik.model, data,
                                e.middleRows(cnt, task->dimension()));
            VLOG(10) << "J = " << J;
            task->compute_jacobian(ik.model, data,
                                   J.middleRows(cnt, task->dimension()));
            cnt += task->dimension();
        }

        if (visitor.should_stop(ik, dq)) {
            return q;
        }

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

        apply_joint_clipping(ik.model, q);

        // If issues, perform random restart
    }

    return q0;
    // If it didn't converge, try a random restart?
}

}  // namespace ik