
#include "cink/Solver.hpp"

#include <pinocchio/parsers/urdf.hpp>

#include "cink/barriers/SelfCollision.hpp"
#include "cink/limits/Configuration.hpp"
#include "cink/limits/Velocity.hpp"
#include "cink/tasks/CentreOfMass.hpp"
#include "cink/tasks/Damping.hpp"
#include "cink/tasks/Frame.hpp"
#include "cink/tasks/Posture.hpp"

int main(int argc, char **argv) {
    // Load a model
    const std::string urdf_filename = "./ur_description/urdf/ur5_robot.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    auto pmodel = std::make_shared<pinocchio::Model>(model);
    auto pdata = std::make_shared<pinocchio::Data>(model);

    // Create geometry data
    auto pgmodel = std::make_shared<pinocchio::GeometryModel>();
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION,
                               *pgmodel, "./ur_description/");
    pgmodel->addAllCollisionPairs();

    auto pgdata = std::make_shared<pinocchio::GeometryData>(*pgmodel);

    // Create configuration
    auto cfg = cink::Configuration(
        pmodel, pdata, Eigen::VectorXd::Zero(model.nq), pgmodel, pgdata);

    cink::String ee_frame = "tool0";
    auto q = pinocchio::randomConfiguration(model);

    cfg.update(q);

    // Create a solver
    auto solver = cink::InverseKinematicsSolver();

    // Create a random task
    auto frame_task = std::make_shared<cink::FrameTask>(ee_frame);

    frame_task->setTargetFromConfiguration(cfg);
    pinocchio::SE3 T = frame_task->getTarget();
    T.translation().x() += 0.01;
    frame_task->setTarget(T);
    frame_task->setOrientationCost(1e-1);
    frame_task->setLevenbergMarquardtDamping(1e-1);

    auto q_limit = std::make_shared<cink::ConfigurationLimit>(
        cfg, Eigen::MatrixXd::Identity(model.nv, model.nv));

    auto v_limit = std::make_shared<cink::VelocityLimit>(
        cfg, Eigen::MatrixXd::Identity(model.nv, model.nv), model.velocityLimit,
        -model.velocityLimit);

    v_limit->setLimitGain(0.9);

    auto com = std::make_shared<cink::CentreOfMassTask>();
    com->setTargetFromConfiguration(cfg);

    auto self_collisions =
        std::make_shared<cink::SelfCollisionBarrier>(cfg, 10);

    solver.addTask(frame_task);
    solver.addTask(com);
    solver.addLimit(q_limit);
    solver.addLimit(v_limit);
    // solver.addBarrier(self_collisions);

    cink::QPSolver::Options options;
    options["printLevel"] = "none";
    solver.init(cfg, "qpoases", options);

    // Compute the new error

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