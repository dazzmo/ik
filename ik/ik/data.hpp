#pragma once

#include "ik/common.hpp"
#include "ik/problem.hpp"

namespace ik {

class problem_data {
   public:
    problem_data() = default;

    problem_data(const InverseKinematicsProblem &problem);

    // Pinocchio model data for the associated model
    pinocchio::Data model_data;

    // Whether the program was successful or not
    bool success = false;

    // Configuration vector iterate
    vector_t q;
    // Tangent-space step direction
    vector_t dq;

    // Task errors partitioned by priority
    std::vector<vector_t> e;
    // Task jacobian partitioned by priority
    std::vector<matrix_t> J;
};

void evaluate_problem_data(InverseKinematicsProblem &problem,
                           problem_data &data);

}  // namespace ik
