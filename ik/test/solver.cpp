
#include "ik/Solver.hpp"

#include <pinocchio/parsers/urdf.hpp>

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

    std::cout << cfg.getTransformFrameToWorld(ee_frame);

    // Create a random task
    auto frame_task = std::make_shared<ik::FrameTask>(ee_frame);
    frame_task->setTargetFromConfiguration(cfg);

    auto T = frame_task->getTarget();
    T.translation().y() += 0.1;
    frame_task->setTarget(T);

    // Create a solver
    auto solver = ik::InverseKinematicsSolver();
    solver.addTask(frame_task);
    solver.init(cfg);

    frame_task->setOrientationCost(0.0);

    // Compute the new error

    Eigen::VectorXd e(6);
    std::cout << cfg.configuration() << std::endl;

    auto dv = solver.solve(cfg);
    cfg.integrateInPlace(dv, 1.0);
    std::cout << cfg.configuration() << std::endl;
    frame_task->computeError(cfg, e);
    std::cout << e << std::endl;

    dv = solver.solve(cfg);
    cfg.integrateInPlace(dv, 1.0);
    std::cout << cfg.configuration() << std::endl;
    frame_task->computeError(cfg, e);
    std::cout << e << std::endl;

    return 0;
}