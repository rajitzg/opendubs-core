#include "ll_control/pixhawk_processing_node.hpp"

namespace ll_control {

PixhawkProcessingNode::PixhawkProcessingNode() : Node("pixhawk_processing_node") {
    this->declare_parameter<std::string>("input_pose_topic", "pose_topic");
    this->declare_parameter<std::string>("input_velocity_topic", "velocity_topic");
    this->declare_parameter<std::string>("output_odometry_topic", "odometry_topic");

    processed_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        this->get_parameter("output_odometry_topic").as_string(), 10);

    pose_sub_.subscribe(this, this->get_parameter("input_pose_topic").as_string());
    vel_sub_.subscribe(this, this->get_parameter("input_velocity_topic").as_string());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), pose_sub_, vel_sub_);

    sync_->registerCallback(std::bind(
        &PixhawkProcessingNode::odometryCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
}

PixhawkProcessingNode::~PixhawkProcessingNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Pixhawk Processing Node.");
}

void PixhawkProcessingNode::odometryCallback(
            const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose,
            const nav_msgs::msg::Odometry::ConstSharedPtr vel) {
    nav_msgs::msg::Odometry processed_odom;

    processed_odom.header = pose->header;
    processed_odom.child_frame_id = vel->child_frame_id;
    processed_odom.pose.pose = pose->pose;
    processed_odom.twist = vel->twist;
    processed_odom_pub_->publish(processed_odom);
}

}  // namespace ll_control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ll_control::PixhawkProcessingNode>());
    rclcpp::shutdown();
    return 0;
}