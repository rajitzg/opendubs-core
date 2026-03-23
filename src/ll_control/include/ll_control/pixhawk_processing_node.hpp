#ifndef PIXHAWK_PROCESSING_NODE_HPP_
#define PIXHAWK_PROCESSING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace ll_control {

    class PixhawkProcessingNode : public rclpp::Node {
    public:
        PixhawkProcessingNode();
        virtual ~PixhawkProcessingNode();
    private:
        void odometryCallback(
            const geometry_msgs::msg::PoseStamped::SharedPtr pose,
            const nav_msgs::msg::Odometry::SharedPtr vel
        );
    }
}