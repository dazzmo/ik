#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class URDFLoaderNode {
   public:
    /**
     * @brief Loads a model defined by URDF in the ROS2 node under the
     * robot_description parameter. Creates a joint state publisher and TF
     * broadcaster for the floating base coordinates, if applicable.
     *
     * @param node Node that the robot description is a parameter on.
     * @param floating_base Whether the model possesses a floating base.
     */
    URDFLoaderNode(const rclcpp::Node::SharedPtr& node,
                   const bool floating_base = false,
                   const std::string& root_frame_name = "base_link");

    /**
     * @brief Set the configuration of the model with generalised coordinates q.
     *
     * @param q
     */
    void setConfiguration(const Eigen::Ref<const Eigen::VectorXd>& q);

    /**
     * @brief Publishes the state of the model to the robot/joint_states topic.
     *
     */
    void publish();

   private:
    bool floating_base;
    rclcpp::Node::SharedPtr node_;
    sensor_msgs::msg::JointState joint_state_msg_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // Number of joints in the model
    std::size_t n_joints_;

    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
    // Name of the root link of the model to attach the floating base frame to
    std::string root_link_name_;
    // Frames to attach to the model to create a floating base
    geometry_msgs::msg::TransformStamped rigid_link_frame_;
    geometry_msgs::msg::TransformStamped floating_base_frame_;
};
