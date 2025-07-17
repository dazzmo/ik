#include "ik/Solver.hpp"

namespace ik {

void InverseKinematicsSolver::init(const Configuration &cfg,
                                   const String &solver) {
    // Create quadratic program
    // todo - add up any hard constraints

    qp_ = std::make_unique<QPSolver>(cfg.nv(), 0, solver);

    is_init_ = true;
}

InverseKinematicsSolver::Vector InverseKinematicsSolver::solve(
    const Configuration &cfg) {
    if (!is_init_) {
        assert("Solver has not been initialised!");
    }

    // Construct inverse kinematics
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(cfg.nv(), cfg.nv());
    Eigen::VectorXd g = Eigen::VectorXd::Zero(cfg.nv());

    Eigen::MatrixXd A;

    Eigen::VectorXd ubA;
    Eigen::VectorXd lbA;

    Eigen::VectorXd ubx;
    Eigen::VectorXd lbx;

    for (const auto &task : tasks_) {
        // Determine the error and Jacobian
        task->addToQPObjective(cfg, H, g);
    }

    ubx = cfg.model().velocityLimit;
    lbx = -cfg.model().velocityLimit;

    std::cout << H << std::endl;
    std::cout << g << std::endl;
    std::cout << ubx << std::endl;
    std::cout << lbx << std::endl;

    // Solve
    qp_->solve(H, g, A, ubA, lbA, ubx, lbx);

    // Return the change in velocity needed
    return qp_->getPrimalSolution();
};

}  // namespace ik