#include "cink/Solver.hpp"

namespace cink {

void InverseKinematicsSolver::init(const Configuration &cfg,
                                   const String &solver,
                                   const QPSolver::Options &opts) {
    // Create quadratic program
    // Determine size of A matrix
    Size nc = 0;
    for (const auto &task : tasks_) {
        if (task->isConstraint()) nc += task->getDimension();
    }
    for (const auto &limit : limits_) {
        nc += limit->getDimension();
    }
    for (const auto &barrier : barriers_) {
        nc += barrier->getDimension();
    }

    qp_ = std::make_unique<QPSolver>(cfg.nv(), nc, solver, opts);

    // Create data
    data_ = std::make_unique<Data>(cfg.nv(), nc);

    is_init_ = true;
}

InverseKinematicsSolver::Vector InverseKinematicsSolver::solve(
    const Configuration &cfg, const Real &dt) {
    if (!is_init_) {
        assert("Solver has not been initialised!");
    }

    data_->reset();

    Size cidx = 0;

    for (const auto &task : tasks_) {
        // Determine the error and Jacobian
        if (task->isConstraint()) {
            Size m = task->getDimension();
            task->computeQPConstraints(cfg, data_->A.middleRows(cidx, m),
                                       data_->lbA.middleRows(cidx, m),
                                       data_->ubA.middleRows(cidx, m));
        } else {
            task->addToQPObjective(cfg, data_->H, data_->g);
        }
    }

    for (const auto &limit : limits_) {
        // Determine the error and Jacobian
        Size m = limit->getDimension();
        limit->computeQPConstraints(cfg, data_->A.middleRows(cidx, m),
                                    data_->lbA.middleRows(cidx, m),
                                    data_->ubA.middleRows(cidx, m), dt);
        cidx += m;
    }

    for (const auto &barrier : barriers_) {
        // Determine the error and Jacobian
        Size m = barrier->getDimension();
        barrier->computeQPConstraints(cfg, data_->A.middleRows(cidx, m),
                                      data_->lbA.middleRows(cidx, m),
                                      data_->ubA.middleRows(cidx, m), dt);
        cidx += m;
    }

    // Add damping
    data_->H.diagonal().array() += damping_;

    // Solve
    qp_->solve(data_->H, data_->g, data_->A, data_->ubA, data_->lbA,
               cfg.model().velocityLimit, -cfg.model().velocityLimit);
    // Return the change in velocity needed
    return qp_->getPrimalSolution() / dt;
};

}  // namespace cink