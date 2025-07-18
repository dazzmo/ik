
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

    pinocchio::SE3 T = frame_task->getTarget();
    T.translation().x() += 0.01;
    frame_task->setTarget(T);

    // Create a solver
    auto solver = ik::InverseKinematicsSolver();
    solver.addTask(frame_task);
    ik::QPSolver::Options options;
    solver.init(cfg, "qpoases", options);

    frame_task->setOrientationCost(1e-1);

    // Compute the new error

    std::cout << cfg.configuration() << std::endl;

    // for (int i = 0; i < 20; ++i) {
    auto dv = solver.solve(cfg);

    std::cout << "dv = " << dv << std::endl;

    // Assess the error
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::VectorXd e = Eigen::VectorXd::Zero(6);
    frame_task->computeError(cfg, e);
    frame_task->computeJacobian(cfg, J);

    std::cout << e << std::endl;
    std::cout << "w " << frame_task->getWeighting() << std::endl;
    std::cout << (J * dv + e).transpose() *
                     frame_task->getWeighting().asDiagonal() * (J * dv + e)
              << std::endl;
    cfg.integrateInPlace(dv, 1.0);

    frame_task->computeError(cfg, e);
    std::cout << e << std::endl;

    return 0;
}