#include "ik_ros/rviz_model_loader.hpp"

URDFLoaderNode::URDFLoaderNode(const rclcpp::Node::SharedPtr& node,
                               const bool floating_base,
                               const std::string& root_frame_name)
    : node_(node),
      floating_base(floating_base),
      n_joints_(0),
      root_link_name_(root_frame_name) {
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
        if (floating_base) {
            pinocchio::urdf::buildModelFromXML(
                urdf_content, pinocchio::JointModelFreeFlyer(), model);
        } else {
            pinocchio::urdf::buildModelFromXML(urdf_content, model);
        }
        RCLCPP_INFO(node->get_logger(), "Pinocchio model successfully loaded!");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "Error loading URDF into Pinocchio: %s", e.what());
    }

    // Set the number of joints for the system
    n_joints_ = floating_base ? model.nq - 7 : model.nq;

    // Initialise the joint state message
    for (std::size_t i = 2; i < model.names.size(); ++i) {
        joint_state_msg_.name.push_back(model.names[i]);
        joint_state_msg_.position.push_back(0.0);
    }

    // Create topics to visualise the model
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/robot/joint_states", 10);

    // Transform broadcaster
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

void URDFLoaderNode::setConfiguration(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
    // Joint states
    this->joint_state_msg_.header.frame_id = "world";
    int start_idx = 0;
    if (floating_base) start_idx = 7;
    for (int i = 0; i < n_joints_; ++i) {
        joint_state_msg_.position[i] = q[start_idx + i];
    }

    // If a floating base exists
    if (floating_base) {
        // Rigidly connect the floating base frame to the root link
        rigid_link_frame_.transform.translation.x = 0.0;
        rigid_link_frame_.transform.translation.y = 0.0;
        rigid_link_frame_.transform.translation.z = 0.0;
        rigid_link_frame_.transform.rotation.w = 1.0;
        rigid_link_frame_.transform.rotation.x = 0.0;
        rigid_link_frame_.transform.rotation.y = 0.0;
        rigid_link_frame_.transform.rotation.z = 0.0;
        rigid_link_frame_.header.frame_id = "floating_base";
        rigid_link_frame_.child_frame_id = root_link_name_;

        // Set the odometry frame to the transform representative of the
        // base configuration in SE3
        floating_base_frame_.transform.translation.x = q[0];
        floating_base_frame_.transform.translation.y = q[1];
        floating_base_frame_.transform.translation.z = q[2];
        floating_base_frame_.transform.rotation.x = q[3];
        floating_base_frame_.transform.rotation.y = q[4];
        floating_base_frame_.transform.rotation.z = q[5];
        floating_base_frame_.transform.rotation.w = q[6];
        floating_base_frame_.header.frame_id = "world";
        floating_base_frame_.child_frame_id = "floating_base";
    }
}

void URDFLoaderNode::publish() {
    this->joint_state_msg_.header.stamp = node_->now();
    this->joint_state_pub_->publish(joint_state_msg_);
    // If a floating base exists
    rigid_link_frame_.header.stamp = node_->now();
    floating_base_frame_.header.stamp = node_->now();
    br_->sendTransform(rigid_link_frame_);
    br_->sendTransform(floating_base_frame_);
}
