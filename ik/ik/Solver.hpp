#pragma once

#include <casadi/casadi.hpp>

#include "ik/QPSolver.hpp"
#include "ik/Task.hpp"

namespace ik {

class InverseKinematicsSolver {
   public:
    using Vector = Eigen::VectorX<Real>;

    using TaskPtr = std::shared_ptr<TaskAbstract>;

    InverseKinematicsSolver() : qp_(nullptr) {}

    void addTask(const TaskPtr &task) { tasks_.push_back(task); }

    void addTasks(const std::vector<TaskPtr> &tasks) {
        for (const auto &task : tasks) {
            addTask(task);
        }
    }

    void clearTasks() { tasks_.clear(); }

    void init(const Configuration &cfg, const String &solver = "qpoases",
              const QPSolver::Options &opts = {});

    Vector solve(const Configuration &cfg);

   private:
    bool is_init_;
    std::vector<TaskPtr> tasks_;

    std::unique_ptr<QPSolver> qp_;
};

}  // namespace ik
