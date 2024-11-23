#pragma once

#include <bopt/program.hpp>
#include <bopt/solvers/qpoases.hpp>

#include "ik/frame_task.hpp"
#include "ik/task.hpp"

namespace ik {

class InverseKinematicsProblem {
   public:
    InverseKinematicsProblem(model_t &model) : model(model) {}

    void init() {}

    std::size_t e_size() {
        std::size_t sz = 0;
        for (const auto &task : get_all_tasks()) {
            sz += task->dimension();
        }
        return sz;
    }

    std::shared_ptr<FrameTask> add_frame_task(
        const std::string &name, const std::string &target_frame,
        const FrameTask::Type &type = FrameTask::Type::Full,
        const std::string &reference_frame = "universe") {
        // Insert into map
        frame_tasks_map_.insert({name, frame_tasks_.size()});
        // Create shared frame task
        frame_tasks_.push_back(std::make_shared<FrameTask>(
            model, target_frame, type, reference_frame));
        // Return pointer to task
        return frame_tasks_.back();
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
        const string_t &reference_frame = "universe") {
        com_task_ = std::make_shared<CentreOfMassTask>(reference_frame);
        return com_task_;
    }

    std::shared_ptr<CentreOfMassTask> get_centre_of_mass_task() {
        return com_task_;
    }

    std::vector<std::shared_ptr<Task>> get_all_tasks() {
        std::vector<std::shared_ptr<Task>> tasks = {};
        tasks.insert(tasks.end(), frame_tasks_.begin(), frame_tasks_.end());
        if (com_task_) {
            tasks.push_back(com_task_);
        }
        return tasks;
    }

    model_t model;

   private:
    std::shared_ptr<CentreOfMassTask> com_task_ = nullptr;

    std::vector<std::shared_ptr<FrameTask>> frame_tasks_;
    std::unordered_map<string_t, std::size_t> frame_tasks_map_;
};

}  // namespace ik
