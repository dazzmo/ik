
#include "ik/ik.hpp"
#include "ik/dls.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>

TEST(ik, AddPositionTask) {
    // Load a model
    const std::string urdf_filename = "ur5.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    std::shared_ptr<ik::PositionTask> task =
        std::make_shared<ik::PositionTask>(model, "ee_fixed_joint", "universe");

    ik::ik ik(model);
    ik.add_position_task("ee", task);
    ik.get_position_task("ee")->reference.position << 1.0, 0.0, 0.0;
    
    // Solve
    dls(ik, pinocchio::randomConfiguration(model));

    ik.get_position_task("ee")->reference.position << 0.5, 0.5, 0.5;
    dls(ik, pinocchio::randomConfiguration(model));
    
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