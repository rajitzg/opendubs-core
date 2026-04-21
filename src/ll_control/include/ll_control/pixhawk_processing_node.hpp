#ifndef PIXHAWK_PROCESSING_NODE_HPP_
#define PIXHAWK_PROCESSING_NODE_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ll_control
{

class PixhawkProcessingNode : public rclcpp::Node
{
public:
  PixhawkProcessingNode();
  ~PixhawkProcessingNode() override;

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::PoseStamped, nav_msgs::msg::Odometry>;

  void odometryCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose,
    const nav_msgs::msg::Odometry::ConstSharedPtr vel);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr processed_odom_pub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> vel_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace ll_control

#endif  // PIXHAWK_PROCESSING_NODE_HPP_
