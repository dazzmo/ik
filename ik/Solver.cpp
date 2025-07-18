#include "ik/Solver.hpp"

namespace ik {

void InverseKinematicsSolver::init(const Configuration &cfg,
                                   const String &solver,
                                   const QPSolver::Options &opts) {
    // Create quadratic program
    // Determine size of A matrix
    Size nc = 0;
    for (const auto &limit : limits_) {
        nc += limit->getDimension();
    }

    qp_ = std::make_unique<QPSolver>(cfg.nv(), nc, solver, opts);

    is_init_ = true;
}

InverseKinematicsSolver::Vector InverseKinematicsSolver::solve(
    const Configuration &cfg, const Real &dt) {
    if (!is_init_) {
        assert("Solver has not been initialised!");
    }

    // Construct inverse kinematics
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(qp_->nx(), qp_->nx());
    Eigen::VectorXd g = Eigen::VectorXd::Zero(qp_->nx());

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(qp_->nc(), qp_->nx());

    Eigen::VectorXd ubA = Eigen::VectorXd::Zero(qp_->nc());
    Eigen::VectorXd lbA = Eigen::VectorXd::Zero(qp_->nc());

    Eigen::VectorXd ubx = cfg.model().velocityLimit;
    Eigen::VectorXd lbx = -cfg.model().velocityLimit;

    for (const auto &task : tasks_) {
        // Determine the error and Jacobian
        task->addToQPObjective(cfg, H, g);
    }

    Size cidx = 0;
    for (const auto &limit : limits_) {
        // Determine the error and Jacobian
        Size m = limit->getDimension();
        limit->computeQPConstraints(cfg, A.middleRows(cidx, m),
                                    ubA.middleRows(cidx, m),
                                    lbA.middleRows(cidx, m), dt);
        cidx += m;
    }

    // Add damping
    H.diagonal().array() += damping_;

    // Solve
    qp_->solve(H, g, A, ubA, lbA, ubx, lbx);
    // Return the change in velocity needed
    return qp_->getPrimalSolution();
};

}  // namespace ik