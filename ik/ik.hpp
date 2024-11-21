#pragma once

#include <bopt/program.hpp>
#include <bopt/solvers/qpoases.hpp>

#include "ik/task.hpp"

namespace ik {

/**
 * @brief Visitor class that can be customised and added to a typical ik
 * instance
 *
 */
// class ik_visitor {
//     void init();
//     void update_references();
//     void update_parameters();
// };

struct ik_task_maps {
    template <typename TaskType>
    using task_map_t = std::unordered_map<string_t, std::shared_ptr<TaskType>>;

    task_map_t<PositionTask> position_tasks_;
    // task_map_t<OrientationTask> orientation_tasks_;
    // task_map_t<SE3Task> se3_tasks_;
    // task_map_t<Task> generic_tasks_;
};

class ik {
   public:
    typedef state<eigen_vector_t> model_state_t;

    ik(model_t &model) : model(model) {}

    void init() {
        // All tasks added, finalise dynamics constraint
        // Create solver
        is_initialised_ = true;
    }

    std::size_t e_size() {
        std::size_t sz = 0;
        for (const auto &task : get_position_tasks()) {
            sz += task.second->dimension();
        }
        // for (const auto &task : get_orientation_tasks()) {
        //     sz += task.second->dimension();
        // }
        // for (const auto &task : get_se3_tasks()) {
        //     sz += task.second->dimension();
        // }
        return sz;
    }

    void add_position_task(const std::string &name,
                           std::shared_ptr<PositionTask> &task) {
        tasks_.position_tasks_.insert({name, task});
        add_task_to_program(*task);
    }

    std::shared_ptr<PositionTask> get_position_task(const string_t &name) {
        if (tasks_.position_tasks_.find(name) != tasks_.position_tasks_.end()) {
            return tasks_.position_tasks_.at(name);
        }
        return nullptr;
    }

    ik_task_maps::task_map_t<PositionTask> &get_position_tasks() {
        return tasks_.position_tasks_;
    }

    // ik_task_maps::task_map_t<OrientationTask> &get_orientation_tasks() {
    //     tasks_.orientation_tasks_;
    // }

    // ik_task_maps::task_map_t<SE3Task> &get_se3_tasks() { tasks_.se3_tasks_; }

    // void add_orientation_task(const std::string &name,
    //                           std::shared_ptr<OrientationTask> &task) {
    //     tasks_.orientation_tasks_.insert({name, task});
    //     add_task_to_program(*task);
    // }

    // std::shared_ptr<OrientationTask> get_orientation_task(
    //     const string_t &name) {
    //     if (tasks_.orientation_tasks_.find(name) !=
    //         tasks_.orientation_tasks_.end()) {
    //         return tasks_.orientation_tasks_.at(name);
    //     }
    //     return nullptr;
    // }

    // void add_se3_task(const std::string &name, std::shared_ptr<SE3Task>
    // &task) {
    //     tasks_.se3_tasks_.insert({name, task});
    //     add_task_to_program(*task);
    // }

    // std::shared_ptr<SE3Task> get_se3_task(const string_t &name) {
    //     if (tasks_.se3_tasks_.find(name) != tasks_.se3_tasks_.end()) {
    //         return tasks_.se3_tasks_.at(name);
    //     }
    //     return nullptr;
    // }

    template <class TaskType>
    void add_task_to_program(const TaskType &task) {}

    // void add_holonomic_constraint_forces_to_program(
    //     const HolonomicConstraint &constraint) {
    //     // Create new variables
    //     eigen_vector_var_t lambda =
    //         create_variable_vector("lambda", constraint.dimension());

    //     variables.lambda.conservativeResize(variables.lambda.size() +
    //                                         lambda.size());
    //     variables.lambda.bottomRows(lambda.size()) = lambda;

    //     // Add constraint to dynamics
    //     dynamics_.add_constraint(constraint);

    //     // Add lamba to variables
    //     for (std::size_t i = 0; i < lambda.size(); ++i) {
    //         program.add_variable(lambda[i]);
    //     }
    // }

    template <class TaskType>
    void update_task(const model_t &model_state, TaskType &task) {
        // Set weighting
        // program.set_parameter(task.parameters().w, task.parameters().w);
        typedef typename task_traits<TaskType>::task_state_t task_state_t;
        typedef typename task_traits<TaskType>::task_error_t task_error_t;

        task_state_t task_state;
        // Evaluate task error
        task.get_task_state(model_state, task_state);
        task_error_t task_error(task.dimension());
        task.get_task_error(task_state, task_error);
    }

    model_t &model;

   private:
    // Task map
    ik_task_maps tasks_;

    // Symbolic model

    // Initialisation flag
    bool is_initialised_ = false;
};

template <class TaskType>
typename task_traits<TaskType>::task_error_t get_task_error(
    const model_t &model, const data_t &data, TaskType &task) {
    // Set weighting
    // program.set_parameter(task.parameters().w, task.parameters().w);
    typedef typename task_traits<TaskType>::task_state_t task_state_t;
    typedef typename task_traits<TaskType>::task_error_t task_error_t;

    task_state_t task_state;
    // Evaluate task error
    task.get_task_state(model, data, task_state);
    task_error_t task_error;
    task.get_task_error(task_state, task_error);
    return task_error;
}

}  // namespace ik
