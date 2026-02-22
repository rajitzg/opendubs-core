#ifndef LL_CONTROL_NODE_HPP_
#define LL_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"
#include "ll_control/pid_controller.hpp"
#include <cstdlib>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace ll_control {

    // ControlMode mirrors the values published by teleop_interface on /teleop/control_mode
    enum class ControlMode : int8_t {
        MANUAL   = 0,
        VELOCITY = 1,
        AUTO     = 2
    };

    class LLControlNode : public rclcpp::Node {
    public:
        LLControlNode();
        virtual ~LLControlNode();

    private:
        // Callbacks
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void modeCallback(const std_msgs::msg::Int8::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void controlLoop();

        // Diagnostics
        void publishInputStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
        void publishSensorStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

        // Helper
        void setArduPilotMode(const std::string& mode);

        // Parameter Callback
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        // Subscribers & Publishers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

        // Internal state
        ControlMode current_mode_{ControlMode::MANUAL};
        geometry_msgs::msg::Twist current_cmd_vel_;  // latest setpoint from teleop
        nav_msgs::msg::Odometry current_odom_;

        // PID / Velocity Control State
        PidController pid_yaw_;
        PidController pid_x_;
        PidController pid_y_;

        double current_yaw_rate_{0.0};
        double prev_control_loop_time_{0.0};

        // Output mapping (Mecanum Mixer inputs: Fwd, Strafe, Turn)
        int output_ch_fwd_{8};
        int output_ch_lat_{9};
        int output_ch_turn_{10};

        // Safety / Timeout
        rclcpp::Time last_cmd_vel_time_;
        rclcpp::Time last_imu_msg_time_;
        rclcpp::Time last_freq_fix_time_;
        double current_imu_freq_{0.0};
        bool odom_received_{false};

        double cmd_vel_timeout_threshold_{0.5}; 
        double imu_timeout_threshold_{0.2};
        double imu_fix_period_{2.0};
        double imu_freq_threshold_{0.9};
        double imu_target_freq_{50.0};

        std::string mode_hold_name_{"HOLD"};

        bool debug_mode{false};

        // Diagnostics
        diagnostic_updater::Updater updater_{this};
    };

} // namespace ll_control

#endif // LL_CONTROL_NODE_HPP_
