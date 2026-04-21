#include <chrono>
#include <memory>
#include <string>

#include "mavros_msgs/srv/command_long.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using CommandLong = mavros_msgs::srv::CommandLong;

// MAVLink MAV_CMD_DO_SET_SERVO
static constexpr uint16_t MAV_CMD_DO_SET_SERVO = 183;

// goBILDA RGB Indicator PWM range (µs)
static constexpr double PWM_MIN = 1000.0;  // off / minimum valid pulse
static constexpr double PWM_MAX = 1950.0;  // full brightness, end of hue sweep

class OpticalFlowLedNode : public rclcpp::Node
{
public:
  OpticalFlowLedNode() : Node("optical_flow_LED_node")
  {
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description =
        "LED brightness [0.0 = off, 1.0 = full]. "
        "Maps linearly to PWM 1000-1950 µs.";
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value = 0.0;
      range.to_value = 1.0;
      range.step = 0.0;
      desc.floating_point_range.push_back(range);
      declare_parameter("brightness", 1.0, desc);
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description =
        "ArduPilot servo output channel (1-16) the LED is wired to. "
        "Set SERVOn_FUNCTION=0 on the FCU for this channel.";
      declare_parameter("servo_channel", 9, desc);
    }
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = "Rate (Hz) at which the PWM command is re-sent.";
      declare_parameter("publish_rate", 1.0, desc);
    }

    // Read initial values
    brightness_ = get_parameter("brightness").as_double();
    servo_channel_ = get_parameter("servo_channel").as_int();
    publish_rate_ = get_parameter("publish_rate").as_double();

    // Service client
    cmd_client_ = create_client<CommandLong>("/mavros/cmd/command");

    // Parameter-change callback
    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) { return on_param_change(params); });

    // Periodic timer
    start_timer();

    RCLCPP_INFO(
      get_logger(), "Optical Flow LED node started. Channel=%d, Brightness=%.2f → PWM=%.0f µs",
      servo_channel_, brightness_, brightness_to_pwm(brightness_));
  }

private:
  static double brightness_to_pwm(double brightness)
  {
    brightness = std::clamp(brightness, 0.0, 1.0);
    return PWM_MIN + brightness * (PWM_MAX - PWM_MIN);
  }

  void start_timer()
  {
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { timer_cb(); });
  }

  // Parameter callback

  rcl_interfaces::msg::SetParametersResult on_param_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      if (p.get_name() == "brightness") {
        brightness_ = p.as_double();
        RCLCPP_INFO(
          get_logger(), "Brightness updated to %.2f → PWM=%.0f µs", brightness_,
          brightness_to_pwm(brightness_));

      } else if (p.get_name() == "servo_channel") {
        servo_channel_ = static_cast<int>(p.as_int());
        RCLCPP_INFO(get_logger(), "Servo channel updated to %d", servo_channel_);

      } else if (p.get_name() == "publish_rate") {
        publish_rate_ = p.as_double();
        timer_->cancel();
        start_timer();
        RCLCPP_INFO(get_logger(), "Publish rate updated to %.1f Hz", publish_rate_);
      }
    }

    return result;
  }

  // Timer callback

  void timer_cb()
  {
    if (!cmd_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "/mavros/cmd/command service not available yet – retrying…");
      return;
    }

    double pwm = brightness_to_pwm(brightness_);

    auto req = std::make_shared<CommandLong::Request>();
    req->broadcast = false;
    req->command = MAV_CMD_DO_SET_SERVO;
    req->confirmation = 0;
    // MAV_CMD_DO_SET_SERVO:
    //   param1 = servo channel (1-based)
    //   param2 = PWM value in µs
    req->param1 = static_cast<float>(servo_channel_);
    req->param2 = static_cast<float>(pwm);
    req->param3 = 0.0f;
    req->param4 = 0.0f;
    req->param5 = 0.0f;
    req->param6 = 0.0f;
    req->param7 = 0.0f;

    cmd_client_->async_send_request(req, [this](rclcpp::Client<CommandLong>::SharedFuture future) {
      auto resp = future.get();
      if (!resp->success) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "DO_SET_SERVO command rejected by FCU (result=%d)",
          resp->result);
      }
    });
  }

  double brightness_;
  int servo_channel_;
  double publish_rate_;

  rclcpp::Client<CommandLong>::SharedPtr cmd_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpticalFlowLedNode>());
  rclcpp::shutdown();
  return 0;
}
