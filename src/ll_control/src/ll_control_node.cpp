#include "ll_control/ll_control_node.hpp"

namespace ll_control {

LLControlNode::LLControlNode() : Node("ll_control_node") {
    // Declare and get parameters
    debug_mode = this->declare_parameter("debug_mode", debug_mode);

    // Output Mapping
    output_ch_fwd_ = this->declare_parameter("output_ch_fwd", output_ch_fwd_);
    output_ch_lat_ = this->declare_parameter("output_ch_lat", output_ch_lat_);
    output_ch_turn_ = this->declare_parameter("output_ch_turn", output_ch_turn_);

    // PID / Limits

    // X Velocity
    PidConfig cfg_x;
    cfg_x.max_output = this->declare_parameter("pid_vel_x.max_output", 1.0);
    cfg_x.k_p = this->declare_parameter("pid_vel_x.k_p", 0.0);
    cfg_x.k_i = this->declare_parameter("pid_vel_x.k_i", 0.0);
    cfg_x.k_d = this->declare_parameter("pid_vel_x.k_d", 0.0);
    cfg_x.k_ff = this->declare_parameter("pid_vel_x.k_ff", 0.0);
    pid_x_.configure(cfg_x);

    // Y Velocity
    PidConfig cfg_y;
    cfg_y.max_output = this->declare_parameter("pid_vel_y.max_output", 1.0);
    cfg_y.k_p = this->declare_parameter("pid_vel_y.k_p", 0.0);
    cfg_y.k_i = this->declare_parameter("pid_vel_y.k_i", 0.0);
    cfg_y.k_d = this->declare_parameter("pid_vel_y.k_d", 0.0);
    cfg_y.k_ff = this->declare_parameter("pid_vel_y.k_ff", 0.0);
    pid_y_.configure(cfg_y);

    // Yaw Rate
    PidConfig cfg_yaw;
    cfg_yaw.max_output = this->declare_parameter("pid_yaw_rate.max_output", 1.57);
    cfg_yaw.k_p = this->declare_parameter("pid_yaw_rate.k_p", 1.0);
    cfg_yaw.k_i = this->declare_parameter("pid_yaw_rate.k_i", 0.1);
    cfg_yaw.k_d = this->declare_parameter("pid_yaw_rate.k_d", 0.0);
    cfg_yaw.k_ff = this->declare_parameter("pid_yaw_rate.k_ff", 0.0);
    pid_yaw_.configure(cfg_yaw);

    // Initialize PID Debugging
    pid_x_.init(this, "vel_x");
    pid_y_.init(this, "vel_y");
    pid_yaw_.init(this, "yaw_rate");

    // Safety
    cmd_vel_timeout_threshold_ = this->declare_parameter("cmd_vel_timeout_threshold", cmd_vel_timeout_threshold_);
    imu_timeout_threshold_ = this->declare_parameter("imu_timeout_threshold", imu_timeout_threshold_);
    imu_fix_period_ = this->declare_parameter("imu_fix_period", imu_fix_period_);
    imu_freq_threshold_ = this->declare_parameter("imu_freq_threshold", imu_freq_threshold_);
    imu_target_freq_ = this->declare_parameter("imu_target_freq", imu_target_freq_);
    mode_hold_name_ = this->declare_parameter("mode_hold_name", mode_hold_name_);

    // Subscriptions
    // Twist setpoints from teleop_interface (latch on RELIABLE QoS isn't needed; 
    // we use a generous keep-last depth so we don't miss a mode change)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/teleop/cmd_vel",
        10,
        std::bind(&LLControlNode::cmdVelCallback, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/teleop/control_mode",
        rclcpp::QoS(10).reliable().transient_local(), // transient_local so we get the last mode on startup
        std::bind(&LLControlNode::modeCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::SensorDataQoS(),
        std::bind(&LLControlNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data",
        rclcpp::SensorDataQoS(),
        std::bind(&LLControlNode::imuCallback, this, std::placeholders::_1));

    // Publisher
    rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
        "/mavros/rc/override", 10);

    // Service client
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // Initialize State
    last_cmd_vel_time_ = this->now();
    last_imu_msg_time_ = this->now();
    last_freq_fix_time_ = this->now();
    prev_control_loop_time_ = this->now().seconds();

    // Control loop timer (50 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&LLControlNode::controlLoop, this));

    updater_.setHardwareID("ll_controller");
    updater_.add("Input Status", this, &LLControlNode::publishInputStatus);
    updater_.add("Sensor Status", this, &LLControlNode::publishSensorStatus);

    // Parameter Callback
    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LLControlNode::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LL Control Node initialized.");

    // Debug Mode Warning
    if (debug_mode) {
        RCLCPP_WARN(this->get_logger(), "Debug mode enabled: ArduPilot mode changes are disabled.");
    }
}

LLControlNode::~LLControlNode() {
    RCLCPP_WARN(this->get_logger(), "Shutting down LL Control Node. Switching to HOLD and resetting RC Overrides to Neutral (1500).");

    setArduPilotMode(mode_hold_name_);

    auto msg = mavros_msgs::msg::OverrideRCIn();
    std::fill(msg.channels.begin(), msg.channels.end(), 65535);
    
    // Set outputs to Neutral (1500)
    msg.channels[output_ch_fwd_] = 1500;
    msg.channels[output_ch_lat_] = 1500;
    msg.channels[output_ch_turn_] = 1500;
    
    rc_override_pub_->publish(msg);
    
    // Give it a moment to publish
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void LLControlNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    current_cmd_vel_ = *msg;
    last_cmd_vel_time_ = this->now();
}

void LLControlNode::modeCallback(const std_msgs::msg::Int8::SharedPtr msg) {
    ControlMode new_mode = static_cast<ControlMode>(msg->data);

    if (new_mode != current_mode_) {
        RCLCPP_INFO(this->get_logger(), "Mode received: %d (0=MAN, 1=VEL, 2=AUTO)", msg->data);
        current_mode_ = new_mode;

        // Reset PIDs on entry to VELOCITY
        if (current_mode_ == ControlMode::VELOCITY) {
            pid_yaw_.reset();
            pid_x_.reset();
            pid_y_.reset();
        }
    }
}

void LLControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // TODO: implement odom callback
    (void)msg;
    odom_received_ = true;
}

void LLControlNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time now = this->now();

    current_yaw_rate_ = msg->angular_velocity.z;

    double dt = (now - last_imu_msg_time_).seconds();
    if (dt > 0.0001) {
        double inst_freq = 1.0 / dt;
        current_imu_freq_ = (0.95 * current_imu_freq_) + (0.05 * inst_freq);
    }
    last_imu_msg_time_ = now;
}

void LLControlNode::publishInputStatus(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    double cmd_vel_age = (this->now() - last_cmd_vel_time_).seconds();
    if (cmd_vel_age > cmd_vel_timeout_threshold_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "cmd_vel Timeout");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
    }
    stat.add("cmd_vel Age (s)", cmd_vel_age);
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
    std::fill(msg.channels.begin(), msg.channels.end(), 65535);

    double now_sec = this->now().seconds();

    // cmd_vel timeout: teleop_interface stopped publishing (controller disconnect or node crashed)
    bool cmd_vel_timeout = (now_sec - last_cmd_vel_time_.seconds()) > cmd_vel_timeout_threshold_;

    bool imu_timeout = (now_sec - last_imu_msg_time_.seconds()) > imu_timeout_threshold_;
    bool imu_slow = current_imu_freq_ < imu_target_freq_ * imu_freq_threshold_;

    if (cmd_vel_timeout) {
        RCLCPP_ERROR_SKIPFIRST_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "CRITICAL: cmd_vel Lost (controller disconnected or teleop stopped)! Switching to HOLD.");
        setArduPilotMode(mode_hold_name_);
        
        // Output Neutral
        msg.channels[output_ch_fwd_] = 1500;
        msg.channels[output_ch_lat_] = 1500;
        msg.channels[output_ch_turn_] = 1500;
        rc_override_pub_->publish(msg);
        return;
    }

    if (current_mode_ != ControlMode::MANUAL) {
        if (imu_timeout) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: IMU Lost! Switching to HOLD.");
            setArduPilotMode(mode_hold_name_);
            
            // Output Neutral
            msg.channels[output_ch_fwd_] = 1500;
            msg.channels[output_ch_lat_] = 1500;
            msg.channels[output_ch_turn_] = 1500;
            rc_override_pub_->publish(msg);
            return;
        } else if (imu_slow) {
            if (now_sec - last_freq_fix_time_.seconds() > imu_fix_period_) {
                last_freq_fix_time_ = this->now();
                RCLCPP_WARN(this->get_logger(), "IMU rate low: %.1f Hz. Attempting to fix rate.", current_imu_freq_);
                std::thread([this]() {
                    std::string cmd = "ros2 run mavros mav sys rate --all " + std::to_string(static_cast<int>(this->imu_target_freq_));
                    std::system(cmd.c_str());
                }).detach();
            }
        }
    }

    uint16_t out_fwd = 1500;
    uint16_t out_lat = 1500;
    uint16_t out_yaw = 1500;

    // Calculate DT for control loop
    double dt = now_sec - prev_control_loop_time_;
    if (dt <= 0 || dt > 1.0) dt = 1.0 / imu_target_freq_;
    prev_control_loop_time_ = now_sec;

    switch (current_mode_) {
        case ControlMode::MANUAL:
            // Twist values are already normalized (-1..1) from teleop_interface
            out_fwd = static_cast<uint16_t>(1500 + static_cast<int>(current_cmd_vel_.linear.x  * 500.0));
            out_lat = static_cast<uint16_t>(1500 + static_cast<int>(current_cmd_vel_.linear.y  * 500.0));
            out_yaw = static_cast<uint16_t>(1500 + static_cast<int>(current_cmd_vel_.angular.z * 500.0));
            break;

        case ControlMode::VELOCITY: // pass through
        case ControlMode::AUTO:
            double fwd_effort = pid_x_.calculate(
                current_cmd_vel_.linear.x,  0.0 /*No Odom yet*/, dt);
            double lat_effort = pid_y_.calculate(
                current_cmd_vel_.linear.y,  0.0 /*No Odom yet*/, dt);
            double yaw_effort = pid_yaw_.calculate(
                current_cmd_vel_.angular.z, current_yaw_rate_,   dt);

            out_fwd = static_cast<uint16_t>(1500 + static_cast<int>(fwd_effort * 500.0));
            out_lat = static_cast<uint16_t>(1500 + static_cast<int>(lat_effort * 500.0));
            out_yaw = static_cast<uint16_t>(1500 + static_cast<int>(yaw_effort * 500.0));
            break;
    }

    // Assign to mapped output channels
    if (output_ch_fwd_ < 18) msg.channels[output_ch_fwd_] = out_fwd;
    if (output_ch_lat_ < 18) msg.channels[output_ch_lat_] = out_lat;
    if (output_ch_turn_ < 18) msg.channels[output_ch_turn_] = out_yaw;

    rc_override_pub_->publish(msg);
}

void LLControlNode::setArduPilotMode(const std::string& mode) {
    // In debug mode, we skip actually sending the SetMode request to avoid issues when MAVROS isn't running.
    if (debug_mode) {
        RCLCPP_INFO_SKIPFIRST_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Debug mode enabled: skipping SetMode request to '%s'",
            mode.c_str());
        return;
    }

    if (!set_mode_client_->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_ERROR(this->get_logger(), "SetMode service not available");
        return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;

    // Async call to avoid blocking main thread too long
    using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture;
    auto response_received_callback = [this, mode](ServiceResponseFuture future) {
        auto result = future.get();
        if (result->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "SetArduPilotMode successful: %s", mode.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "SetArduPilotMode failed: %s", mode.c_str());
        }
    };

    set_mode_client_->async_send_request(request, response_received_callback);
}

rcl_interfaces::msg::SetParametersResult LLControlNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    PidConfig cfg_x   = pid_x_.getConfig();   bool update_x   = false;
    PidConfig cfg_y   = pid_y_.getConfig();   bool update_y   = false;
    PidConfig cfg_yaw = pid_yaw_.getConfig(); bool update_yaw = false;


    for (const auto &param : parameters) {
        std::string name = param.get_name();

        // PID X
        if      (name == "pid_vel_x.k_p")        { cfg_x.k_p        = param.as_double(); update_x   = true; }
        else if (name == "pid_vel_x.k_i")        { cfg_x.k_i        = param.as_double(); update_x   = true; }
        else if (name == "pid_vel_x.k_d")        { cfg_x.k_d        = param.as_double(); update_x   = true; }
        else if (name == "pid_vel_x.k_ff")       { cfg_x.k_ff       = param.as_double(); update_x   = true; }
        else if (name == "pid_vel_x.max_output") { cfg_x.max_output = param.as_double(); update_x   = true; }
        
        // PID Y
        else if (name == "pid_vel_y.k_p")        { cfg_y.k_p        = param.as_double(); update_y   = true; }
        else if (name == "pid_vel_y.k_i")        { cfg_y.k_i        = param.as_double(); update_y   = true; }
        else if (name == "pid_vel_y.k_d")        { cfg_y.k_d        = param.as_double(); update_y   = true; }
        else if (name == "pid_vel_y.k_ff")       { cfg_y.k_ff       = param.as_double(); update_y   = true; }
        else if (name == "pid_vel_y.max_output") { cfg_y.max_output = param.as_double(); update_y   = true; }
        
        // PID Yaw
        else if (name == "pid_yaw_rate.k_p")        { cfg_yaw.k_p        = param.as_double(); update_yaw = true; }
        else if (name == "pid_yaw_rate.k_i")        { cfg_yaw.k_i        = param.as_double(); update_yaw = true; }
        else if (name == "pid_yaw_rate.k_d")        { cfg_yaw.k_d        = param.as_double(); update_yaw = true; }
        else if (name == "pid_yaw_rate.k_ff")       { cfg_yaw.k_ff       = param.as_double(); update_yaw = true; }
        else if (name == "pid_yaw_rate.max_output") { cfg_yaw.max_output = param.as_double(); update_yaw = true; }
        
        // Safety
        else if (name == "cmd_vel_timeout_threshold") { cmd_vel_timeout_threshold_ = param.as_double(); }
        else if (name == "imu_timeout_threshold")     { imu_timeout_threshold_     = param.as_double(); }
        else if (name == "imu_fix_period")            { imu_fix_period_            = param.as_double(); }
        else if (name == "imu_target_freq")           { imu_target_freq_           = param.as_double(); }
    }

    if (update_x)   { pid_x_.configure(cfg_x);     RCLCPP_INFO(this->get_logger(), "Updated PID X: P=%.2f I=%.2f D=%.2f FF=%.2f",   cfg_x.k_p,   cfg_x.k_i,   cfg_x.k_d,   cfg_x.k_ff);   }
    if (update_y)   { pid_y_.configure(cfg_y);     RCLCPP_INFO(this->get_logger(), "Updated PID Y: P=%.2f I=%.2f D=%.2f FF=%.2f",   cfg_y.k_p,   cfg_y.k_i,   cfg_y.k_d,   cfg_y.k_ff);   }
    if (update_yaw) { pid_yaw_.configure(cfg_yaw); RCLCPP_INFO(this->get_logger(), "Updated PID Yaw: P=%.2f I=%.2f D=%.2f FF=%.2f", cfg_yaw.k_p, cfg_yaw.k_i, cfg_yaw.k_d, cfg_yaw.k_ff); }

    return result;
}

} // namespace ll_control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ll_control::LLControlNode>());
    rclcpp::shutdown();
    return 0;
}
