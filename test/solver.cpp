
#include "ik/Solver.hpp"

#include <pinocchio/parsers/urdf.hpp>

#include "ik/limits/Configuration.hpp"
#include "ik/limits/Velocity.hpp"
#include "ik/tasks/Frame.hpp"

int main(int argc, char **argv) {
    // Load a model
    const std::string urdf_filename = "ur5.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    auto pmodel = std::make_shared<pinocchio::Model>(model);
    auto pdata = std::make_shared<pinocchio::Data>(model);

    // Create configuration
    auto cfg =
        ik::Configuration(pmodel, pdata, Eigen::VectorXd::Zero(model.nq));

    ik::String ee_frame = "tool0";

    auto q = pinocchio::randomConfiguration(model);

    cfg.update(q);

    // Create a solver
    auto solver = ik::InverseKinematicsSolver();

    // Create a random task
    auto frame_task = std::make_shared<ik::FrameTask>(ee_frame);

    frame_task->setTargetFromConfiguration(cfg);
    pinocchio::SE3 T = frame_task->getTarget();
    T.translation().x() += 0.01;
    frame_task->setTarget(T);
    frame_task->setOrientationCost(1e-1);
    frame_task->setLevenbergMarquardtDamping(1e-1);

    auto q_limit = std::make_shared<ik::ConfigurationLimit>(
        cfg, Eigen::MatrixXd::Identity(model.nv, model.nv));

    auto v_limit = std::make_shared<ik::VelocityLimit>(
        cfg, Eigen::MatrixXd::Identity(model.nv, model.nv), model.velocityLimit,
        -model.velocityLimit);

    v_limit->setLimitGain(0.9);

    solver.addTask(frame_task);
    solver.addLimit(q_limit);
    solver.addLimit(v_limit);

    ik::QPSolver::Options options;
    options["printLevel"] = "none";
    solver.init(cfg, "qpoases", options);

    // Compute the new error

    std::cout << cfg.configuration() << std::endl;

    solver.setDamping(1e-6);
    for (int i = 0; i < 20; ++i) {
        auto dv = solver.solve(cfg, 0.1);
        cfg.integrateInPlace(dv, 0.1);
    }
    // Assess the error
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(6);
    frame_task->computeError(cfg, e);

    std::cout << e << std::endl;

    return 0;
}