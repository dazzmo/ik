#include "ik/dls.hpp"

namespace ik {

vector_t dls(InverseKinematicsProblem &problem, const vector_t &q0,
                    dls_data &data, const inverse_kinematics_visitor &visitor,
                    const dls_parameters &p) {
    data.q = q0;

    // todo - if limited convergence, try random walk
    auto constraints = problem.get_all_constraints();

    // Perform iterations
    for (index_t i = 0; i < p.max_iterations; ++i) {
        // Updata all program data
        evaluate_problem_data(problem, data);

        index_t cnt = 0;
        // Get all priority 0 tasks
        for (index_t p = 0; p <= problem.max_priority_level(); ++p) {
            data.et.middleRows(cnt, data.e[p].rows()) << data.e[p];
            data.Jt.middleRows(cnt, data.J[p].rows()) << data.J[p];
            cnt += data.e[p].rows();
        }

        cnt = 0;
        for (auto &constraint : constraints) {
            // Get references
            matrix_ref_t Ji = data.Jc.middleRows(cnt, constraint->dimension());
            // Compute constraint error and jacobians
            constraint->compute_jacobian(problem.model(), data.model_data, Ji);
            // Move to next constraint
            cnt += constraint->dimension();
        }

        // Compute J J^T
        // todo - create an efficient version of this which doesn't required
        // copying
        data.JJ.noalias() = data.Jt * data.Jt.transpose();
        // Damp
        data.JJ.diagonal().array() += p.damping * p.damping;

        // Project into constraint null-space
        data.N.setIdentity();
        if (data.Jc.rows() > 0) {
            data.N -=
                data.Jc.completeOrthogonalDecomposition().pseudoInverse() *
                data.Jc;
        }

        // Compute step direction
        data.dq =
            -data.N * (data.Jt.transpose() * data.JJ.ldlt().solve(data.et));

        VLOG(10) << "dls: it = " << i;
        VLOG(10) << "dls: e = " << data.et.transpose();
        VLOG(10) << "dls: q = " << data.q.transpose();
        VLOG(10) << "dls: dq = " << data.dq.transpose();
        VLOG(15) << "dls: J = " << data.Jt;

        if (visitor.should_stop(problem, data.e, data.dq)) {
            data.success = true;
            return data.q;
        }

        // Take a step
        data.q = pinocchio::integrate(problem.model(), data.q,
                                      p.step_length * data.dq);

        // Clip joints if any exceed bounds
        apply_joint_clipping(problem.model(), data.q);

        // If issues, perform random restart
    }

    data.success = false;
    return data.q;
}

}  // namespace ik