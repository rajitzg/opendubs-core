#ifndef LL_CONTROL_PID_CONTROLLER_HPP_
#define LL_CONTROL_PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace ll_control {

struct PidConfig {
    double k_p = 0.0;
    double k_i = 0.0;
    double k_d = 0.0;
    double k_ff = 0.0;
    double max_output = 1.0;     // Max physical velocity (m/s or rad/s)
    double integrity_limit = 1.0; // Anti-windup limit
    double deadband = 0.05;       // Normalized input deadband
};

struct PidState {
    double integral = 0.0;
    double prev_error = 0.0;
};

class PidController {
public:
    PidController() = default;

    void init(rclcpp::Node* node, const std::string& name_prefix) {
        if (!node) return;
        
        std::string topic_base = "ll_control/debug/" + name_prefix;
        pub_target_ = node->create_publisher<std_msgs::msg::Float64>(topic_base + "/target", 10);
        pub_measured_ = node->create_publisher<std_msgs::msg::Float64>(topic_base + "/measured", 10);
        pub_measured_ = node->create_publisher<std_msgs::msg::Float64>(topic_base + "/measured", 10);
        pub_effort_ = node->create_publisher<std_msgs::msg::Float64>(topic_base + "/effort", 10);
        
        initialized_ = true;
    }

    void configure(const PidConfig& config) {
        config_ = config;
    }

    void reset() {
        state_ = PidState();
    }

    /**
     * @brief Calculate control output (Normalized -1.0 to 1.0)
     * 
     * @param input_norm Normalized command input (-1.0 to 1.0)
     * @param feedback Current system state (e.g. gyro rate or velocity)
     * @param dt Time delta in seconds
     * @return double Control effort (-1.0 to 1.0)
     */
    double calculate(double input_norm, double feedback, double dt) {
        // 1. Apply Deadband and Map to Target
        double cmd = applyDeadband(input_norm, config_.deadband);
        double target = cmd * config_.max_output;
        
        // Publish Debug Info
        if (initialized_) {
            std_msgs::msg::Float64 msg;
            msg.data = target;
            pub_target_->publish(msg);
            
            msg.data = feedback;
            pub_measured_->publish(msg);
        }

        // 3. Compute Error
        double error = target - feedback;

        // 4. Update Integral
        state_.integral += error * dt;
        state_.integral = std::clamp(state_.integral, -config_.integrity_limit, config_.integrity_limit);

        // 5. Derivative
        double derivative = 0.0;
        if (dt > 0.000001) {
            derivative = (error - state_.prev_error) / dt;
        }
        state_.prev_error = error;

        // 6. Output (PID)
        double output = (config_.k_p * error) + 
                        (config_.k_i * state_.integral) + 
                        (config_.k_d * derivative);

        // 7. Feedforward
        output += (target * config_.k_ff);

        // 8. Clamp to Normalized Effort (-1.0 to 1.0)
        double final_output = std::clamp(output, -1.0, 1.0);
        
        if (initialized_) {
            std_msgs::msg::Float64 msg;
            msg.data = final_output;
            pub_effort_->publish(msg);
        }
        
        return final_output;
    }

    // Accessors for debugging
    const PidState& getState() const { return state_; }
    const PidConfig& getConfig() const { return config_; }

private:
    PidConfig config_;
    PidState state_;
    
    // Debugging
    bool initialized_{false};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_measured_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_effort_;

    double applyDeadband(double input, double threshold) {
        if (std::abs(input) < threshold) {
            return 0.0;
        }
        // Rescale so it starts at 0 right after deadband
        if (input > 0) {
            return (input - threshold) / (1.0 - threshold);
        } else {
            return (input + threshold) / (1.0 - threshold);
        }
    }
};

} // namespace ll_control

#endif // LL_CONTROL_PID_CONTROLLER_HPP_
