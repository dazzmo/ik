
#include "ik/task.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(Task, Construct) {
    const std::string urdf_filename = "ur5.urdf";
    // Load the urdf model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    ik::PositionTask task(model, "ee_fixed_joint", "universe");
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);

    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    FLAGS_log_prefix = 1;
    // FLAGS_v = 10;

    int status = RUN_ALL_TESTS();

    bopt::profiler summary;
    return status;
}