#include <ik/dls.hpp>
#include <ik/program.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class URDFLoaderNode {
   public:
    URDFLoaderNode(const rclcpp::Node::SharedPtr& node) : node_(node) {
        // Get the robot description parameter (commonly "robot_description")
        std::string urdf_param = "robot_description";
        std::string urdf_content;

        // Check if the parameter exists
        node->declare_parameter(urdf_param, "");  // defaults to ""
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

        // Initialise the joint state message
        for (std::size_t i = 1; i < model.names.size(); ++i) {
            joint_state_msg_.name.push_back(model.names[i]);
            joint_state_msg_.position.push_back(0.0);
        }

        // Create topics to visualise the model
        joint_state_pub_ =
            node_->create_publisher<sensor_msgs::msg::JointState>(
                "/robot/joint_states", 10);
    }

    void setConfiguration(const Eigen::Ref<const Eigen::VectorXd>& q) {
        // Joint states
        this->joint_state_msg_.header.frame_id = "world";
        for (int i = 0; i < q.size(); ++i) {
            joint_state_msg_.position[i] = q[i];
        }

        // If a floating base exists
        // // Rigidly connect the floating base frame to the root link
        // rigid_link_frame_.transform.translation.x = 0.0;
        // rigid_link_frame_.transform.translation.y = 0.0;
        // rigid_link_frame_.transform.translation.z = 0.0;
        // rigid_link_frame_.transform.rotation.w = 1.0;
        // rigid_link_frame_.transform.rotation.x = 0.0;
        // rigid_link_frame_.transform.rotation.y = 0.0;
        // rigid_link_frame_.transform.rotation.z = 0.0;
        // rigid_link_frame_.header.frame_id = "floating_base";
        // rigid_link_frame_.child_frame_id = "root_link";

        // // Set the odometry frame to the transform representative of the base
        // // configuration in SE3
        // Eigen::Vector3d pos(q[0], q[1], q[2]);
        // Eigen::Quaterniond rot(q[6], q[3], q[4], q[5]);
        // floating_base_frame_.transform.translation = toVector3(pos);
        // floating_base_frame_.transform.rotation = toQuaternion(rot);
        // floating_base_frame_.header.frame_id = "world";
        // floating_base_frame_.child_frame_id = "floating_base";
    }

    void publish() {
        this->joint_state_msg_.header.stamp = node_->now();
        this->joint_state_pub_->publish(joint_state_msg_);
        // If a floating base exists
        // rigid_link_frame_.header.stamp = node_->now();
        // floating_base_frame_.header.stamp = node_->now();
        // br_->sendTransform(rigid_link_frame_);
        // br_->sendTransform(floating_base_frame_);
    }

   private:
    rclcpp::Node::SharedPtr node_;
    sensor_msgs::msg::JointState joint_state_msg_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

class URHandler {
   public:
    void init(const rclcpp::Node::SharedPtr& node) {
        urdf_ = std::make_unique<URDFLoaderNode>(node);

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
        ik_->add_frame_task("tip", "wrist_3_joint",
                            ik::FrameTask::Type::Position);
        ik_->add_centre_of_mass_task();
    }

    void loop(const rclcpp::Node::SharedPtr& node) {
        ik_->get_frame_task("tip")->target.translation() << 0.5,
            0.5 * cos(node->now().seconds()),
            0.25 + 0.5 * sin(node->now().seconds());

        // Compute inverse kinematics solution
        Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
        ik::dls_parameters p;
        p.damping = 1e-1;
        q = ik::dls(*ik_, q, ik::inverse_kinematics_visitor(), p);
        urdf_->setConfiguration(q);
    }

    void publish() { urdf_->publish(); }

   private:
    // Position
    std::unique_ptr<ik::InverseKinematicsProblem> ik_;
    std::unique_ptr<URDFLoaderNode> urdf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(100);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ik");

    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    FLAGS_log_prefix = 1;
    FLAGS_v = 10;

    URHandler urdf;

    urdf.init(node);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        urdf.loop(node);
        urdf.publish();

        loop_rate.sleep();
    }

    return 0;
}