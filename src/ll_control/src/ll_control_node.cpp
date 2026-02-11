#include "ll_control/ll_control_node.hpp"

namespace ll_control {

LLControlNode::LLControlNode() : Node("ll_control_node") {
    // Declare and get parameters
    
    // Mode Switching (Strings)
    mode_map_manual_ = this->declare_parameter("mode_map_manual", mode_map_manual_);
    mode_map_velocity_ = this->declare_parameter("mode_map_velocity", mode_map_velocity_);
    mode_map_auto_ = this->declare_parameter("mode_map_auto", mode_map_auto_);

    // Input Mapping
    input_ch_fwd_ = this->declare_parameter("input_ch_fwd", input_ch_fwd_);
    input_ch_lat_ = this->declare_parameter("input_ch_lat", input_ch_lat_);
    input_ch_yaw_ = this->declare_parameter("input_ch_yaw", input_ch_yaw_);
    
    // Output Mapping
    output_ch_fwd_ = this->declare_parameter("output_ch_fwd", output_ch_fwd_);
    output_ch_lat_ = this->declare_parameter("output_ch_lat", output_ch_lat_);
    output_ch_turn_ = this->declare_parameter("output_ch_turn", output_ch_turn_);

    // Deadband
    input_deadband_ = this->declare_parameter("input_deadband", input_deadband_);

    // PID / Limits
    
    // X Velocity
    PidConfig cfg_x;
    cfg_x.max_output = this->declare_parameter("pid_vel_x.max_output", 1.0);
    cfg_x.k_p = this->declare_parameter("pid_vel_x.k_p", 0.0);
    cfg_x.k_i = this->declare_parameter("pid_vel_x.k_i", 0.0);
    cfg_x.k_d = this->declare_parameter("pid_vel_x.k_d", 0.0);
    cfg_x.k_ff = this->declare_parameter("pid_vel_x.k_ff", 0.0);
    cfg_x.deadband = input_deadband_;
    pid_x_.configure(cfg_x);

    // Y Velocity
    PidConfig cfg_y;
    cfg_y.max_output = this->declare_parameter("pid_vel_y.max_output", 1.0);
    cfg_y.k_p = this->declare_parameter("pid_vel_y.k_p", 0.0);
    cfg_y.k_i = this->declare_parameter("pid_vel_y.k_i", 0.0);
    cfg_y.k_d = this->declare_parameter("pid_vel_y.k_d", 0.0);
    cfg_y.k_ff = this->declare_parameter("pid_vel_y.k_ff", 0.0);
    cfg_y.deadband = input_deadband_;
    pid_y_.configure(cfg_y);

    // Yaw Rate
    PidConfig cfg_yaw;
    cfg_yaw.max_output = this->declare_parameter("pid_yaw_rate.max_output", 1.57);
    cfg_yaw.k_p = this->declare_parameter("pid_yaw_rate.k_p", 1.0);
    cfg_yaw.k_i = this->declare_parameter("pid_yaw_rate.k_i", 0.1);
    cfg_yaw.k_d = this->declare_parameter("pid_yaw_rate.k_d", 0.0);
    cfg_yaw.k_ff = this->declare_parameter("pid_yaw_rate.k_ff", 0.0);
    cfg_yaw.deadband = input_deadband_;
    pid_yaw_.configure(cfg_yaw);
    
    // Safety
    rc_timeout_threshold_ = this->declare_parameter("rc_timeout_threshold", rc_timeout_threshold_);
    imu_timeout_threshold_ = this->declare_parameter("imu_timeout_threshold", imu_timeout_threshold_);
    imu_freq_threshold_ = this->declare_parameter("imu_freq_threshold", imu_freq_threshold_);
    imu_target_freq_ = this->declare_parameter("imu_target_freq", imu_target_freq_);

    // Initialize subscribers
    rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
        "/mavros/rc/in", 10, std::bind(&LLControlNode::rcCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, std::bind(&LLControlNode::stateCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 
        rclcpp::SensorDataQoS(), 
        std::bind(&LLControlNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", 
        rclcpp::SensorDataQoS(),
        std::bind(&LLControlNode::imuCallback, this, std::placeholders::_1));

    // Initialize publisher
    rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
        "/mavros/rc/override", 10);

    // Initialize State
    last_rc_msg_time_ = this->now();
    last_imu_msg_time_ = this->now();
    prev_control_loop_time_ = this->now().seconds();

    // Control loop timer (50Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&LLControlNode::controlLoop, this));

    updater_.setHardwareID("ll_controller");
    updater_.add("Input Status", this, &LLControlNode::publishInputStatus);
    updater_.add("Sensor Status", this, &LLControlNode::publishSensorStatus);
        
    RCLCPP_INFO(this->get_logger(), "LL Control Node initialized.");
}

LLControlNode::~LLControlNode() {
    RCLCPP_WARN(this->get_logger(), "Shutting down LL Control Node. Resetting RC Overrides to Neutral (1500).");
    auto msg = mavros_msgs::msg::OverrideRCIn();
    std::fill(msg.channels.begin(), msg.channels.end(), 65535);
    
    // Set outputs to Neutral (1500) instead of Release (0) which would fall back to bad RC Input
    msg.channels[output_ch_fwd_] = 1500;
    msg.channels[output_ch_lat_] = 1500;
    msg.channels[output_ch_turn_] = 1500;
    
    rc_override_pub_->publish(msg);
    // Give it a moment to publish
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void LLControlNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    if (msg->mode == mode_map_manual_) {
        if (current_mode_ != ControlMode::MANUAL) {
            RCLCPP_INFO(this->get_logger(), "Switched to MANUAL mode");
            current_mode_ = ControlMode::MANUAL;
        }
    } else if (msg->mode == mode_map_velocity_) {
        if (current_mode_ != ControlMode::VELOCITY) {
            RCLCPP_INFO(this->get_logger(), "Switched to VELOCITY mode");
            current_mode_ = ControlMode::VELOCITY;
            
            // Reset PID states
            pid_yaw_.reset();
            pid_x_.reset();
            pid_y_.reset();
        }
    } else if (msg->mode == mode_map_auto_) {
        if (current_mode_ != ControlMode::AUTO) {
                    RCLCPP_INFO(this->get_logger(), "Switched to AUTO mode");
            current_mode_ = ControlMode::AUTO;
        }
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Unknown ArduPilot mode: %s", msg->mode.c_str());
    }
}

void LLControlNode::rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg) {
    // Update setpoints from sticks
    if (msg->channels.size() > static_cast<size_t>(std::max({input_ch_fwd_, input_ch_lat_, input_ch_yaw_}))) {
        rc_fwd_ = msg->channels[input_ch_fwd_];
        rc_lat_ = msg->channels[input_ch_lat_];
        rc_yaw_ = msg->channels[input_ch_yaw_];

        if (msg->channels[2] != 900) { // throttle channel
            last_rc_msg_time_ = this->now();
        }
    }
}

void LLControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // TODO: implement odom callback
    // add watchdog check + diagnostics
    (void)msg; // suppress unused warning
    odom_received_ = true;
}

void LLControlNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time now = this->now();
    
    // Store Gyro Z for PID (Z-axis angular velocity)
    // We don't need to manually update state here; the PidController takes 'current' in calculate()
    // But we might want to store it if we used it elsewhere. 
    current_yaw_rate_ = msg->angular_velocity.z; 

    double dt = (now - last_imu_msg_time_).seconds(); 
    if (dt > 0.0001) {
         double inst_freq = 1.0 / dt;
         current_imu_freq_ = (0.95 * current_imu_freq_) + (0.05 * inst_freq);
    }
    last_imu_msg_time_ = now;
}

void LLControlNode::publishInputStatus(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    double rc_age = (this->now() - last_rc_msg_time_).seconds(); 
    if (rc_age > rc_timeout_threshold_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "RC Timeout");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
    }

    stat.add("RC Age (s)", rc_age);
    stat.add("Current Mode", static_cast<int>(current_mode_));
}

void LLControlNode::publishSensorStatus(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    double imu_age = (this->now() - last_imu_msg_time_).seconds();
    if (imu_age > imu_timeout_threshold_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU Timeout");
    } else if (current_imu_freq_ < imu_target_freq_ * imu_freq_threshold_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU Rate Low");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
    }

    stat.add("IMU Age (s)", imu_age);
    stat.add("IMU Freq (hz)", current_imu_freq_);
}

void LLControlNode::controlLoop() {
    auto msg = mavros_msgs::msg::OverrideRCIn();
    
    // Initialize all to UINT16_MAX (ignore)
    std::fill(msg.channels.begin(), msg.channels.end(), 65535);

    double now_sec = this->now().seconds();
    bool rc_timeout = (now_sec - last_rc_msg_time_.seconds()) > rc_timeout_threshold_;
    bool imu_timeout = (now_sec - last_imu_msg_time_.seconds()) > imu_timeout_threshold_;
    bool imu_slow = current_imu_freq_ < imu_target_freq_ * imu_freq_threshold_;

    if (rc_timeout) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "CRITICAL: RC Input Lost! Releasing control.");
        msg.channels[output_ch_fwd_] = 0; 
        msg.channels[output_ch_lat_] = 0;
        msg.channels[output_ch_turn_] = 0;
        rc_override_pub_->publish(msg);
        return; 
    }

    if (current_mode_ != ControlMode::MANUAL) {
        if (imu_timeout) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: IMU Lost! Switching to MANUAL.");
            current_mode_ = ControlMode::MANUAL;
        } else if (imu_slow) {
            RCLCPP_WARN(this->get_logger(), "IMU rate low: %.1f Hz. Switching to MANUAL and attempting to fix rate.", current_imu_freq_);
            std::thread([this]() { 
                std::string cmd = "ros2 run mavros mav sys rate --all " + std::to_string(static_cast<int>(this->imu_target_freq_));
                std::system(cmd.c_str()); 
            }).detach();
            current_mode_ = ControlMode::MANUAL;
        }
    }

    uint16_t out_fwd = 1500;
    uint16_t out_lat = 1500;
    uint16_t out_yaw = 1500;

    // Calculate DT for control loop
    double dt = now_sec - prev_control_loop_time_;
    if (dt <= 0 || dt > 1.0) dt = 1.0 / imu_target_freq_; // Safety fallback
    prev_control_loop_time_ = now_sec;

    switch (current_mode_) {
        case ControlMode::MANUAL:
            out_fwd = rc_fwd_;
            out_lat = rc_lat_;
            out_yaw = rc_yaw_;
            break;
                
        case ControlMode::VELOCITY:
        {
            // Yaw Control
            // 1. Calc Normalized Output
            double yaw_norm_in = (static_cast<double>(rc_yaw_) - 1500.0) / 500.0;
            double yaw_effort = pid_yaw_.calculate(yaw_norm_in, current_yaw_rate_, dt);
            out_yaw = 1500 + static_cast<int>(yaw_effort * 500.0);
            
            // Fwd/Lat (Open Loop for now)
            double fwd_norm_in = (static_cast<double>(rc_fwd_) - 1500.0) / 500.0;
            double fwd_effort = pid_x_.calculate(fwd_norm_in, 0.0 /*No Odom yet*/, dt);
            out_fwd = 1500 + static_cast<int>(fwd_effort * 500.0);

            double lat_norm_in = (static_cast<double>(rc_lat_) - 1500.0) / 500.0;
            double lat_effort = pid_y_.calculate(lat_norm_in, 0.0 /*No Odom yet*/, dt);
            out_lat = 1500 + static_cast<int>(lat_effort * 500.0);
            
            break;
        }
            
        case ControlMode::AUTO:
            // TODO: implement auto logic
            // pwm signals stay at 1500
            break;
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
