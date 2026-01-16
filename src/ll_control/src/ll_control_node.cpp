#include "ll_control/ll_control_node.hpp"

namespace ll_control {

    LLControlNode::LLControlNode() : Node("ll_control_node") {
        // Declare and get parameters (using header defaults)
        mode_channel_ = this->declare_parameter("mode_channel", mode_channel_);
        mode_manual_threshold_ = this->declare_parameter("mode_manual_threshold", mode_manual_threshold_);
        mode_auto_threshold_ = this->declare_parameter("mode_auto_threshold", mode_auto_threshold_);
        
        input_ch_fwd_ = this->declare_parameter("input_ch_fwd", input_ch_fwd_);
        input_ch_lat_ = this->declare_parameter("input_ch_lat", input_ch_lat_);
        input_ch_yaw_ = this->declare_parameter("input_ch_yaw", input_ch_yaw_);
        
        output_ch_fwd_ = this->declare_parameter("output_ch_fwd", output_ch_fwd_);
        output_ch_lat_ = this->declare_parameter("output_ch_lat", output_ch_lat_);
        output_ch_turn_ = this->declare_parameter("output_ch_turn", output_ch_turn_);

        // Initialize subscribers
        rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
            "/mavros/rc/in", 10, std::bind(&LLControlNode::rcCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LLControlNode::odomCallback, this, std::placeholders::_1));

        // Initialize publisher
        rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            "/mavros/rc/override", 10);

        // Initialize safety timer
        last_rc_msg_time_ = this->now();

        // Control loop timer (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&LLControlNode::controlLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "LL Control Node initialized.");
    }

    void LLControlNode::rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg) {

        // use relative node clock to check for RC timeout
        last_rc_msg_time_ = this->now();

        // Mode switching
        if (msg->channels.size() > static_cast<size_t>(mode_channel_)) {
            int mode_pwm = msg->channels[mode_channel_];
            
            if (mode_pwm < mode_manual_threshold_) {
                if (current_mode_ != ControlMode::MANUAL) {
                    RCLCPP_INFO(this->get_logger(), "Switched to MANUAL mode");
                    current_mode_ = ControlMode::MANUAL;
                }
            } else if (mode_pwm >= mode_manual_threshold_ && mode_pwm < mode_auto_threshold_) {
                if (current_mode_ != ControlMode::VELOCITY) {
                    RCLCPP_INFO(this->get_logger(), "Switched to VELOCITY mode");
                    current_mode_ = ControlMode::VELOCITY;
                }
            } else {
                if (current_mode_ != ControlMode::AUTO) {
                    RCLCPP_INFO(this->get_logger(), "Switched to AUTO mode");
                    current_mode_ = ControlMode::AUTO;
                }
            }
        }

        // Update setpoints from sticks
        if (msg->channels.size() > static_cast<size_t>(std::max({input_ch_fwd_, input_ch_lat_, input_ch_yaw_}))) {
            rc_fwd_ = msg->channels[input_ch_fwd_];
            rc_lat_ = msg->channels[input_ch_lat_];
            rc_yaw_ = msg->channels[input_ch_yaw_];
        }
    }

    void LLControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // TODO: implement odom callback
    }

    void LLControlNode::computePID() {
        // TODO: implement PID logic
    }

    void LLControlNode::controlLoop() {
        auto msg = mavros_msgs::msg::OverrideRCIn();
        
        // Initialize all to UINT16_MAX (ignore)
        std::fill(msg.channels.begin(), msg.channels.end(), 65535);

        if ((this->now() - last_rc_msg_time_).seconds() > 1.0) {
            // Release control if RC lost
            msg.channels[output_ch_fwd_] = 0; 
            msg.channels[output_ch_lat_] = 0;
            msg.channels[output_ch_turn_] = 0;
            return; 
        }

        uint16_t out_fwd = 1500;
        uint16_t out_lat = 1500;
        uint16_t out_yaw = 1500;

        if (current_mode_ == ControlMode::MANUAL) {
            // Pass-through
            out_fwd = rc_fwd_;
            out_lat = rc_lat_;
            out_yaw = rc_yaw_;
        } else if (current_mode_ == ControlMode::VELOCITY) {
            computePID();
            out_fwd = rc_fwd_;
            out_lat = rc_lat_;
            out_yaw = rc_yaw_;
        } else if (current_mode_ == ControlMode::AUTO) {
            // TODO: implement auto logic
            // pwm signals will stay at 1500
        }

        // Assign to mapped output channels
        if (output_ch_fwd_ < 18) msg.channels[output_ch_fwd_] = out_fwd;
        if (output_ch_lat_ < 18) msg.channels[output_ch_lat_] = out_lat;
        if (output_ch_turn_ < 18) msg.channels[output_ch_turn_] = out_yaw;

        rc_override_pub_->publish(msg);
    }

} // namespace ll_control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ll_control::LLControlNode>());
    rclcpp::shutdown();
    return 0;
}
