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
    double max_output = 1.0;      // Physical velocity scale (m/s or rad/s).
                                  // Used to normalize FF: (target/max_output)*k_ff.
    double integrity_limit = 1.0; // Anti-windup clamp on the integral state.
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
        pub_target_   = node->create_publisher<std_msgs::msg::Float64>(topic_base + "/target",   10);
        pub_measured_ = node->create_publisher<std_msgs::msg::Float64>(topic_base + "/measured", 10);
        pub_effort_   = node->create_publisher<std_msgs::msg::Float64>(topic_base + "/effort",   10);

        initialized_ = true;
    }

    void configure(const PidConfig& config) {
        config_ = config;
    }

    void reset() {
        state_ = PidState();
    }

    /**
     * @brief Calculate normalized control effort (-1..1).
     *
     * @param target    Desired value in physical units (m/s or rad/s).
     * @param feedback  Current measured value in the same physical units.
     * @param dt        Time delta in seconds.
     * @return double   Normalized control effort clamped to [-1, 1].
     *
     * Deadband is NOT applied here — it is the caller's responsibility to zero
     * small targets before calling (e.g. teleop_interface applies stick deadband).
     */
    double calculate(double target, double feedback, double dt) {
        // 1. Hard-cap the target to the allowed velocity range.
        //    This is the strict motor velocity limit: even if the caller requests more,
        //    the controller will never try to exceed max_output.
        double capped_target = std::clamp(target, -config_.max_output, config_.max_output);

        // 2. Error in physical units against the capped target
        double error = capped_target - feedback;

        // 3. Integral with anti-windup
        state_.integral += error * dt;
        state_.integral = std::clamp(state_.integral,
                                     -config_.integrity_limit,
                                      config_.integrity_limit);

        // 4. Derivative
        double derivative = 0.0;
        if (dt > 0.000001) {
            derivative = (error - state_.prev_error) / dt;
        }
        state_.prev_error = error;

        // 5. PID output (gains map physical error → normalized effort)
        double output = (config_.k_p * error) +
                        (config_.k_i * state_.integral) +
                        (config_.k_d * derivative);

        // 6. Feedforward: normalize capped_target by max_output so k_ff=1 gives
        //    full effort at the velocity cap, regardless of max_output scale.
        output += (capped_target / config_.max_output) * config_.k_ff;

        // 7. Clamp to normalized effort range [-1, 1]
        double final_output = std::clamp(output, -1.0, 1.0);

        // 8. Debug topics
        if (initialized_) {
            std_msgs::msg::Float64 msg;
            msg.data = capped_target; pub_target_->publish(msg);
            msg.data = feedback;      pub_measured_->publish(msg);
            msg.data = final_output;  pub_effort_->publish(msg);
        }

        return final_output;
    }

    const PidState&  getState()  const { return state_; }
    const PidConfig& getConfig() const { return config_; }

private:
    PidConfig config_;
    PidState state_;

    // Debugging
    bool initialized_{false};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_measured_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_effort_;
};

} // namespace ll_control

#endif // LL_CONTROL_PID_CONTROLLER_HPP_
