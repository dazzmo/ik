#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ik/dls.hpp>
#include <ik/problem.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ik_ros/rviz_model_loader.hpp"

class CassieIK {
   public:
    void init(const rclcpp::Node::SharedPtr& node) {
        urdf_ = std::make_unique<URDFLoaderNode>(node, true, "pelvis");

        std::string urdf_param = "robot_description";
        std::string urdf_content;

        // Check if the parameter exists
        while (!node->get_parameter(urdf_param, urdf_content)) {
            RCLCPP_INFO(node->get_logger(), "Waiting for model!");
        }

        // Load the URDF into a Pinocchio model
        pinocchio::Model model;
        try {
            pinocchio::urdf::buildModelFromXML(urdf_content, model);
            RCLCPP_INFO(node->get_logger(),
                        "Pinocchio model successfully loaded!");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(),
                         "Error loading URDF into Pinocchio: %s", e.what());
        }

        ik_ = std::make_unique<ik::InverseKinematicsProblem>(model);
        // Create tasks
        auto fl = ik::FrameTask::create(model, "LeftFootFront",
                                        ik::FrameTask::Type::Position);

        auto fr = ik::FrameTask::create(model, "RightFootFront",
                                        ik::FrameTask::Type::Position);

        auto pelvis =
            ik::FrameTask::create(model, "pelvis", ik::FrameTask::Type::Full);

        q_ = ik::vector_t::Zero(model.nq);
        // Set quaternion
        q_[6] = 1.0;

        ik_->add_frame_task("fl", fl);
        ik_->add_frame_task("fr", fr);
        ik_->add_frame_task("pelvis", pelvis);
    }

    void loop(const rclcpp::Node::SharedPtr& node) {
        ik_->get_frame_task("fl")->target.translation() << 0.1, 0.1,
            -0.6 + 0.3 * sin(node->now().seconds());

        ik_->get_frame_task("fr")->target.translation() << 0.1, 0.1,
            -0.6 - 0.3 * sin(node->now().seconds());

        ik_->get_frame_task("pelvis")->target.translation()
            << 0.5 + 0.5 * sin(node->now().seconds()),
            0, 0;
        ik_->get_frame_task("pelvis")->target.rotation().setIdentity();

        ik::dls_parameters p;
        p.damping = 1e-2;
        // Compute inverse kinematics solution
        q_ = ik::dls(*ik_, q_, ik::inverse_kinematics_visitor(), p);
        // Display
        urdf_->setConfiguration(q_);
    }

    void publish() { urdf_->publish(); }

   private:
    // Position
    std::unique_ptr<ik::InverseKinematicsProblem> ik_;
    std::unique_ptr<URDFLoaderNode> urdf_;
    ik::vector_t q_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(100);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ik");

    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    FLAGS_log_prefix = 1;
    // FLAGS_v = 10;

    CassieIK urdf;

    urdf.init(node);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        urdf.loop(node);
        urdf.publish();

        loop_rate.sleep();
    }

    return 0;
}