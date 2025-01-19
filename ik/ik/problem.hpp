#pragma once

#include "ik/centre_of_mass.hpp"
#include "ik/frame.hpp"
#include "ik/posture.hpp"

namespace ik {

class InverseKinematicsProblem {
   public:
    /**
     * @brief Construct a new Inverse Kinematics Problem based on the
     * kinodynamic model specified by model
     *
     * @param model
     */
    InverseKinematicsProblem(model_t &model,
                             const std::size_t &max_priority_level = 0)
        : model_(model),
          max_priority_level_(max_priority_level),
          tasks_(max_priority_level + 1, std::vector<std::shared_ptr<Task>>()) {
    }

    const std::size_t &max_priority_level() const {
        return max_priority_level_;
    }

    /**
     * @brief Dimension of the error vector within the problem for the requested
     * level of priority.
     *
     * @return std::size_t
     */
    std::size_t e_size(const std::size_t &priority) const {
        std::size_t sz = 0;
        for (const auto &task : get_all_tasks(priority)) {
            sz += task->dimension();
        }
        return sz;
    }

    /**
     * @brief Dimension of the constraint vector within the problem.
     *
     * @return std::size_t
     */
    std::size_t c_size() const {
        std::size_t sz = 0;
        for (const auto &constraint : get_all_constraints()) {
            sz += constraint->dimension();
        }
        return sz;
    }

    std::shared_ptr<FrameTask> add_frame_task(
        const std::string &name, const std::shared_ptr<FrameTask> &task,
        const std::size_t &priority = 0) {
        // Insert into map
        frame_tasks_map_.insert({name, frame_tasks_.size()});
        // Create shared frame task
        frame_tasks_.push_back(task);
        // Add to task vector
        tasks_[priority].push_back(task);
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
            assert("Frame task does not exist!");
            return frame_tasks_.size();
        }
    }

    std::shared_ptr<AlignAxisTask> add_align_axis_task(
        const std::string &name, const std::shared_ptr<AlignAxisTask> &task,
        const std::size_t &priority = 0) {
        // Insert into map
        axis_tasks_map_.insert({name, axis_tasks_.size()});
        // Create shared axis task
        axis_tasks_.push_back(task);
        // Add to task vector
        tasks_[priority].push_back(task);
        // Return pointer to task
        return axis_tasks_.back();
    }

    std::shared_ptr<AlignAxisTask> get_align_axis_task(
        const std::string &name) {
        return axis_tasks_[get_frame_task_index(name)];
    }

    std::size_t get_align_axis_task_index(const string_t &name) {
        if (axis_tasks_map_.find(name) != axis_tasks_map_.end()) {
            return axis_tasks_map_.at(name);
        } else {
            assert("Frame task does not exist!");
            return axis_tasks_map_.size();
        }
    }

    std::shared_ptr<CentreOfMassTask> add_centre_of_mass_task(
        const std::shared_ptr<CentreOfMassTask> &task,
        const std::size_t &priority = 0) {
        com_task_ = task;
        // Add to task vector
        tasks_[priority].push_back(task);
        return com_task_;
    }

    std::shared_ptr<CentreOfMassTask> get_centre_of_mass_task() {
        return com_task_;
    }

    std::shared_ptr<PostureTask> add_posture_task(
        const std::string &name, const std::shared_ptr<PostureTask> &task,
        const std::size_t &priority = 0) {
        // Insert into map
        posture_tasks_map_.insert({name, posture_tasks_.size()});
        // Create shared posture task
        posture_tasks_.push_back(task);
        // Add to task vector
        tasks_[priority].push_back(task);
        // Return pointer to task
        return posture_tasks_.back();
    }

    std::shared_ptr<PostureTask> get_posture_task(const std::string &name) {
        return posture_tasks_[get_posture_task_index(name)];
    }

    std::size_t get_posture_task_index(const string_t &name) {
        if (posture_tasks_map_.find(name) != posture_tasks_map_.end()) {
            return posture_tasks_map_.at(name);
        } else {
            assert("Posture task does not exist!");
            return posture_tasks_.size();
        }
    }

    const std::vector<std::shared_ptr<Task>> &get_all_tasks(
        const std::size_t &priority) const {
        assert(priority <= max_priority_level_ &&
               "Maximum priority level exceeded!");
        return tasks_[priority];
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
    std::size_t max_priority_level_;

    std::shared_ptr<CentreOfMassTask> com_task_ = nullptr;

    // Tasks partitioned by priority
    std::vector<std::vector<std::shared_ptr<Task>>> tasks_;

    // Posture tasks
    std::vector<std::shared_ptr<PostureTask>> posture_tasks_;
    std::unordered_map<string_t, std::size_t> posture_tasks_map_;

    // Frame tasks
    std::vector<std::shared_ptr<FrameTask>> frame_tasks_;
    std::unordered_map<string_t, std::size_t> frame_tasks_map_;

    // Axis alignment tasks
    std::vector<std::shared_ptr<AlignAxisTask>> axis_tasks_;
    std::unordered_map<string_t, std::size_t> axis_tasks_map_;

    // Frame constraints
    std::vector<std::shared_ptr<FrameConstraint>> frame_constraints_;
    std::unordered_map<string_t, std::size_t> frame_constraints_map_;
};

}  // namespace ik
