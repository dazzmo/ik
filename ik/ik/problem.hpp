#pragma once

#include "ik/frame.hpp"
#include "ik/centre_of_mass.hpp"

namespace ik {

class InverseKinematicsProblem {
   public:
    /**
     * @brief Construct a new Inverse Kinematics Problem based on the
     * kinodynamic model specified by model
     *
     * @param model
     */
    InverseKinematicsProblem(model_t &model) : model_(model) {}

    std::size_t e_size() const {
        std::size_t sz = 0;
        for (const auto &task : get_all_tasks()) {
            sz += task->dimension();
        }
        return sz;
    }

    std::size_t c_size() const {
        std::size_t sz = 0;
        for (const auto &constraint : get_all_constraints()) {
            sz += constraint->dimension();
        }
        return sz;
    }

    std::shared_ptr<FrameTask> add_frame_task(
        const std::string &name, const std::shared_ptr<FrameTask> &task) {
        // Insert into map
        frame_tasks_map_.insert({name, frame_tasks_.size()});
        // Create shared frame task
        frame_tasks_.push_back(task);
        // Return pointer to task
        return frame_tasks_.back();
    }

    std::shared_ptr<FrameConstraint> add_frame_constraint(
        const std::string &name,
        const std::shared_ptr<FrameConstraint> &constraint) {
        // Insert into map
        frame_constraints_map_.insert({name, frame_constraints_.size()});
        // Create shared frame constraint
        frame_constraints_.push_back(constraint);
        // Return pointer to constraint
        return frame_constraints_.back();
    }

    std::shared_ptr<FrameTask> get_frame_task(const std::string &name) {
        return frame_tasks_[get_frame_task_index(name)];
    }

    // Vector of frame tasks

    std::size_t get_frame_task_index(const string_t &name) {
        if (frame_tasks_map_.find(name) != frame_tasks_map_.end()) {
            return frame_tasks_map_.at(name);
        } else {
            return frame_tasks_.size();
        }
    }

    std::shared_ptr<CentreOfMassTask> add_centre_of_mass_task(
        const std::shared_ptr<CentreOfMassTask> &task) {
        com_task_ = task;
        return com_task_;
    }

    std::shared_ptr<CentreOfMassTask> get_centre_of_mass_task() {
        return com_task_;
    }

    std::vector<std::shared_ptr<Task>> get_all_tasks() const {
        std::vector<std::shared_ptr<Task>> tasks = {};
        tasks.insert(tasks.end(), frame_tasks_.begin(), frame_tasks_.end());
        if (com_task_) {
            tasks.push_back(com_task_);
        }
        return tasks;
    }

    std::vector<std::shared_ptr<Constraint>> get_all_constraints() const {
        std::vector<std::shared_ptr<Constraint>> constraints = {};
        constraints.insert(constraints.end(), frame_constraints_.begin(),
                           frame_constraints_.end());

        return constraints;
    }

    /**
     * @brief Kinodynamic model the inverse kinematics problem is based on
     *
     * @return const model_t&
     */
    const model_t &model() const { return model_; }

   private:
    model_t model_;
    std::shared_ptr<CentreOfMassTask> com_task_ = nullptr;

    // Frame tasks

    std::vector<std::shared_ptr<FrameTask>> frame_tasks_;
    std::unordered_map<string_t, std::size_t> frame_tasks_map_;

    // Frame constraints

    std::vector<std::shared_ptr<FrameConstraint>> frame_constraints_;
    std::unordered_map<string_t, std::size_t> frame_constraints_map_;
};

}  // namespace ik
