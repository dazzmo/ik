#pragma once

#include <casadi/casadi.hpp>

#include "ik/Limit.hpp"
#include "ik/QPSolver.hpp"
#include "ik/Task.hpp"

namespace ik {

class InverseKinematicsSolver {
   public:
    using Vector = Eigen::VectorX<Real>;

    using TaskPtr = std::shared_ptr<TaskAbstract>;
    using LimitPtr = std::shared_ptr<LimitAbstract>;

    InverseKinematicsSolver() : qp_(nullptr), damping_(0.0) {}

    void init(const Configuration &cfg, const String &solver = "qpoases",
              const QPSolver::Options &opts = {});

    void addTask(const TaskPtr &task) { tasks_.push_back(task); }
    void addTasks(const std::vector<TaskPtr> &tasks) {
        for (const auto &task : tasks) {
            addTask(task);
        }
    }

    void addLimit(const LimitPtr &limit) { limits_.push_back(limit); }
    void addLimits(const std::vector<LimitPtr> &limits) {
        for (const auto &limit : limits) {
            addLimit(limit);
        }
    }

    void clearTasks() { tasks_.clear(); }
    void clearLimits() { limits_.clear(); }

    void setDamping(const Real &value) { damping_ = value; }

    Vector solve(const Configuration &cfg, const Real &dt = 1.0);

   private:
    bool is_init_;
    std::vector<TaskPtr> tasks_;
    std::vector<LimitPtr> limits_;

    Real damping_;
    std::unique_ptr<QPSolver> qp_;
};

}  // namespace ik
