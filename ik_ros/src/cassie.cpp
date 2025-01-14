#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ik/dls.hpp>
#include <ik/pik.hpp>
#include <ik/problem.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ik_ros/rviz_model_loader.hpp"

class CassieIK {
   public:
    enum class IKMethod { DLS = 0, PIK };

    void init(const rclcpp::Node::SharedPtr& node,
              const IKMethod& method = IKMethod::DLS) {
        method_ = method;
        urdf_ = std::make_unique<URDFLoaderNode>(node, true, "base_link");
        std::string urdf_param = "robot_description";
        std::string urdf_content;

        // Check if the parameter exists
        while (!node->get_parameter(urdf_param, urdf_content)) {
            RCLCPP_INFO(node->get_logger(), "Waiting for model!");
        }

        // Load the URDF into a Pinocchio model
        pinocchio::Model model;
        try {
            pinocchio::urdf::buildModelFromXML(
                urdf_content, pinocchio::JointModelFreeFlyer(), model);
            RCLCPP_INFO(node->get_logger(),
                        "Pinocchio model successfully loaded!");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(),
                         "Error loading URDF into Pinocchio: %s", e.what());
        }

        ik_ = std::make_unique<ik::InverseKinematicsProblem>(model, 1);
        // Create leg tasks (with respect to pelvis frame)
        auto fl = ik::FrameTask::create(model, "LeftFootFront",
                                        ik::KinematicType::Position, "pelvis");

        // Frame constraint
        auto fr = ik::FrameTask::create(
            model, "RightFootFront", ik::KinematicType::Position, "universe");

        // Pelvis orientation tracking task in world frame
        auto pelvis =
            ik::FrameTask::create(model, "pelvis", ik::KinematicType::Orientation);

        // Centre of mass task, in world frame
        auto com = ik::CentreOfMassTask::create(model);

        auto posture = ik::PostureTask::create(model, model.nq - 7);
        posture->target.setZero();

        // Create configuration vector
        q_ = ik::vector_t::Zero(model.nq);
        // Set quaternion w component to 1.0
        q_[6] = 1.0;

        // Add a frame task to move the left foot relative to the pelvis
        ik_->add_frame_task("fl", fl, 0);
        // Add a frame constraint to keep the foot in place
        // ik_->add_frame_task("fr", fr);
        // Add a frame task for the pelvis pose within the inertial frame
        ik_->add_frame_task("pelvis", pelvis, 1);
        // ik_->add_posture_task("posture", posture, 1);
        ik_->add_centre_of_mass_task(com, 1);

        // Create data for the program
        if (method_ == IKMethod::DLS) {
            VLOG(10) << "DLS Selected";
            dls_data_ = std::make_unique<ik::dls_data>(*ik_);
        } else if (method_ == IKMethod::PIK) {
            pik_data_ = std::make_unique<ik::pik_data>(*ik_);
        }
    }

    void loop(const rclcpp::Node::SharedPtr& node) {
        double t = node->now().seconds();
        // Very primitive imitation of a walk cycle
        ik_->get_frame_task("fl")->target.translation() << 0.0, 0.1,
            -0.6 + 0.2 * sin(0.5 * t);

        ik_->get_frame_task("pelvis")->target.rotation().setIdentity();
        ik_->get_centre_of_mass_task()->target << 0.0, 0.0, 1.0;

        if (method_ == IKMethod::DLS) {
            VLOG(10) << "DLS Method";
            ik::dls_parameters p;
            // Example parameters
            p.damping = 1e-2;
            p.max_iterations = 100;
            p.step_length = 1e-1;

            // Compute inverse kinematics solution with damped least squares
            q_ = ik::dls(*ik_, q_, *dls_data_, ik::inverse_kinematics_visitor(),
                         p);
        } else if (method_ == IKMethod::PIK) {
            VLOG(10) << "PIK Method";
            ik::pik_parameters p;
            // Example parameters
            p.damping = 1e-2;
            p.max_iterations = 100;
            p.step_length = 1e0;

            // Compute inverse kinematics solution with damped least squares
            q_ = ik::pik(*ik_, q_, *pik_data_, ik::inverse_kinematics_visitor(),
                         p);
        }

        VLOG(10) << "Result: " << q_.transpose();
        // Display
        urdf_->setConfiguration(q_);
    }

    void publish() { urdf_->publish(); }

   private:
    // Position
    std::unique_ptr<URDFLoaderNode> urdf_;

    ik::vector_t q_;
    std::unique_ptr<ik::InverseKinematicsProblem> ik_;
    std::unique_ptr<ik::dls_data> dls_data_;
    std::unique_ptr<ik::pik_data> pik_data_;

    IKMethod method_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(50);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ik");

    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    FLAGS_log_prefix = 1;
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);

    CassieIK urdf;

    urdf.init(node, CassieIK::IKMethod::PIK);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        urdf.loop(node);
        urdf.publish();

        loop_rate.sleep();
    }

    return 0;
}