#ifndef LL_CONTROL_NODE_HPP_
#define LL_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace ll_control
{

enum class ControlMode {
  MANUAL,
  VELOCITY,
  AUTO
};

class LLControlNode : public rclcpp::Node
{
public:
  LLControlNode();
  virtual ~LLControlNode() = default;

private:
  // Callbacks
  void rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // Helper functions
  void computePID();

  // Subscribers & Publishers
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Placeholder for Pinpoint/Odom
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // internal state
  ControlMode current_mode_{ControlMode::MANUAL};
  nav_msgs::msg::Odometry current_odom_;
  
  // RC inputs (raw PWM)
  uint16_t rc_fwd_{1500};
  uint16_t rc_lat_{1500};
  uint16_t rc_yaw_{1500};
  
  // Parameters
  // Mode switch
  int mode_channel_{5}; // default channel 5
  int mode_manual_threshold_{1200};
  int mode_auto_threshold_{1800};

  // Input mapping
  int input_ch_fwd_{1};
  int input_ch_lat_{0};
  int input_ch_yaw_{3};
  
  // Output mapping (Mecanum Mixer inputs: Fwd, Strafe, Turn)
  int output_ch_fwd_{0};
  int output_ch_lat_{1};
  int output_ch_turn_{2};

  // PID placeholders
  double kp_{1.0}; 
  double ki_{0.0};
  double kd_{0.0};
  
  // Safety / Timeout
  rclcpp::Time last_rc_msg_time_;
  bool odom_received_{false};
};

} // namespace ll_control

#endif // LL_CONTROL_NODE_HPP_
