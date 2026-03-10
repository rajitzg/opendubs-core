#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTFBroadcaster : public rclcpp::Node
{
public:
    OdomTFBroadcaster() : Node("odom_tf_broadcaster")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("input_odom_topic", "input_odom");

        // Create the TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Get the topic name from parameters
        std::string odom_topic = this->get_parameter("input_odom_topic").as_string();

        // Create the subscription
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            10,
            std::bind(&OdomTFBroadcaster::odom_callback, this, std::placeholders::_1)
        );
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = this->get_parameter("odom_frame_id").as_string();
        t.child_frame_id = this->get_parameter("base_frame_id").as_string();

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}