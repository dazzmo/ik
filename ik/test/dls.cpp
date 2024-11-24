
#include "ik/problem.hpp"
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
}

TEST(ik, AddOrientationTask) {
    // Load a model
    const std::string urdf_filename = "ur5.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    std::shared_ptr<ik::OrientationTask> task =
        std::make_shared<ik::OrientationTask>(model, "ee_fixed_joint", "universe");

    ik::ik ik(model);
    ik.add_orientation_task("ee", task);
    ik.get_orientation_task("ee")->reference.rotation.setIdentity();
    
    // Solve
    dls(ik, pinocchio::randomConfiguration(model));
}

TEST(ik, AddCentreOfMassTask) {
    // Load a model
    const std::string urdf_filename = "ur5.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    std::shared_ptr<ik::CentreOfMassTask> task =
        std::make_shared<ik::CentreOfMassTask>(model, "universe");

    ik::ik ik(model);
    ik.add_centre_of_mass_task(task);
    ik.get_centre_of_mass_task()->reference.position << 0.0, 0.0, 0.3;
    
    // Solve
    dls(ik, pinocchio::randomConfiguration(model));
}

TEST(ik, AddSE3Task) {
    // Load a model
    const std::string urdf_filename = "ur5.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    std::shared_ptr<ik::SE3Task> task =
        std::make_shared<ik::SE3Task>(model, "ee_fixed_joint", "universe");

    ik::ik ik(model);
    ik.add_se3_task("task", task);
    ik.get_se3_task("task")->reference.pose.setRandom();

    // Solve
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