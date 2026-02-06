#ifndef LL_CONTROL_NODE_HPP_
#define LL_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ll_control/pid_controller.hpp"
#include <cstdlib>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace ll_control {

    enum class ControlMode {
        MANUAL,
        VELOCITY,
        AUTO
    };

    class LLControlNode : public rclcpp::Node {
    public:
        LLControlNode();
        virtual ~LLControlNode() = default;

    private:
        // Callbacks
        void rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void controlLoop();

        // Diagnostics
        void publishInputStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
        void publishSensorStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

        // Subscribers & Publishers
        rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Placeholder for Pinpoint/Odom
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // internal state
        ControlMode current_mode_{ControlMode::MANUAL};
        nav_msgs::msg::Odometry current_odom_;
        
        // RC inputs (raw PWM)
        uint16_t rc_fwd_{1500};
        uint16_t rc_lat_{1500};
        uint16_t rc_yaw_{1500};

        // PID / Velocity Control State
        PidController pid_yaw_;
        PidController pid_x_;
        PidController pid_y_;

        double current_yaw_rate_{0.0}; // Stored for use in controlLoop

        double prev_control_loop_time_{0.0};
        
        // Parameters
        // Mode switch via State string
        std::string mode_map_manual_{"MANUAL"};
        std::string mode_map_velocity_{"ACRO"}; // User can set to STEERING
        std::string mode_map_auto_{"AUTO"};     // User can set to GUIDED/HOLD

        // Input mapping
        int input_ch_fwd_{1};
        int input_ch_lat_{0};
        int input_ch_yaw_{3};
        
        // Output mapping (Mecanum Mixer inputs: Fwd, Strafe, Turn)
        int output_ch_fwd_{0};
        int output_ch_lat_{1};
        int output_ch_turn_{2};

        // Tuning
        double input_deadband_{0.05}; // 5% deadband (normalized 0-1)

        // Safety / Timeout
        rclcpp::Time last_rc_msg_time_;
        rclcpp::Time last_imu_msg_time_;
        double current_imu_freq_{0.0};
        bool odom_received_{false};

        double rc_timeout_threshold_{0.2};
        double imu_timeout_threshold_{0.2};
        double imu_freq_threshold_{0.9};
        double imu_target_freq_{50};

        // Diagnostics
        diagnostic_updater::Updater updater_{this};
    };

} // namespace ll_control

#endif // LL_CONTROL_NODE_HPP_
