#pragma once

#include <bopt/program.hpp>
#include <bopt/solvers/qpoases.hpp>

#include "ik/task.hpp"

namespace ik {

class ik {
   public:
    typedef state<eigen_vector_t> model_state_t;
    typedef Task<double, std::size_t> task_t;

    ik(model_t &model) : model(model) {}

    void init() { is_initialised_ = true; }

    std::size_t e_size() {
        std::size_t sz = 0;
        for (const auto &task : get_all_tasks()) {
            sz += task->dimension();
        }
        return sz;
    }

    void add_position_task(const std::string &name,
                           std::shared_ptr<PositionTask> &task) {
        position_tasks_map_.insert({name, position_tasks_.size()});
        position_tasks_.push_back(task);
    }

    std::shared_ptr<PositionTask> get_position_task(const string_t &name) {
        if (position_tasks_map_.find(name) != position_tasks_map_.end()) {
            return position_tasks_[position_tasks_map_.at(name)];
        }
        return nullptr;
    }

    void add_orientation_task(const std::string &name,
                              std::shared_ptr<OrientationTask> &task) {
        orientation_tasks_map_.insert({name, orientation_tasks_.size()});
        orientation_tasks_.push_back(task);
    }

    std::shared_ptr<OrientationTask> get_orientation_task(
        const string_t &name) {
        if (orientation_tasks_map_.find(name) != orientation_tasks_map_.end()) {
            return orientation_tasks_[orientation_tasks_map_.at(name)];
        }
        return nullptr;
    }

        void add_se3_task(const std::string &name,
                              std::shared_ptr<SE3Task> &task) {
        se3_tasks_map_.insert({name, se3_tasks_.size()});
        se3_tasks_.push_back(task);
    }

    std::shared_ptr<SE3Task> get_se3_task(
        const string_t &name) {
        if (se3_tasks_map_.find(name) != se3_tasks_map_.end()) {
            return se3_tasks_[se3_tasks_map_.at(name)];
        }
        return nullptr;
    }

    void add_centre_of_mass_task(std::shared_ptr<CentreOfMassTask> &task) {
        com_task_ = task;
    }

    std::shared_ptr<CentreOfMassTask> get_centre_of_mass_task() {
        return com_task_;
    }

    std::vector<std::shared_ptr<task_t>> get_all_tasks() {
        std::vector<std::shared_ptr<task_t>> tasks = {};
        tasks.insert(tasks.end(), position_tasks_.begin(),
                     position_tasks_.end());
        tasks.insert(tasks.end(), orientation_tasks_.begin(),
                     orientation_tasks_.end());
        if (com_task_) {
            tasks.push_back(com_task_);
        }
        return tasks;
    }

    model_t &model;

   private:
    std::vector<std::shared_ptr<PositionTask>> position_tasks_;
    std::vector<std::shared_ptr<OrientationTask>> orientation_tasks_;
    std::vector<std::shared_ptr<SE3Task>> se3_tasks_;

    std::shared_ptr<CentreOfMassTask> com_task_ = nullptr;

    std::unordered_map<string_t, std::size_t> position_tasks_map_;
    std::unordered_map<string_t, std::size_t> orientation_tasks_map_;
    std::unordered_map<string_t, std::size_t> se3_tasks_map_;

    // Initialisation flag
    bool is_initialised_ = false;
};

template <class TaskType>
void update_task(const model_t &model, const data_t &data, TaskType &task) {
    // Evaluate task error
    task.compute_task_state(model, data);
    task.compute_task_error(task.state);
}

}  // namespace ik
