#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <chrono>

using namespace std::chrono_literals;


class ServoTestNode : public rclcpp::Node
{
public:
  ServoTestNode() : Node("servo_test_node")
  {
    // Declare parameters with defaults
    this->declare_parameter("servo_num", 1);
    this->declare_parameter("pwm_us", 1500);


    client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");


    // Wait for service, then send once
    timer_ = this->create_wall_timer(500ms, [this]() {
      if (!client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for /mavros/cmd/command service...");
        return;
      }


      int servo_num = this->get_parameter("servo_num").as_int();
      int pwm_us    = this->get_parameter("pwm_us").as_int();


      auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
      request->broadcast    = false;
      request->command      = 183;  // MAV_CMD_DO_SET_SERVO
      request->confirmation = 0;
      request->param1       = static_cast<float>(servo_num);
      request->param2       = static_cast<float>(pwm_us);
      request->param3       = 0;
      request->param4       = 0;
      request->param5       = 0;
      request->param6       = 0;
      request->param7       = 0;


      RCLCPP_INFO(this->get_logger(), "Sending DO_SET_SERVO: output=%d  pwm=%d us", servo_num, pwm_us);


      auto future = client_->async_send_request(
        request,
        [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture f) {
          auto result = f.get();
          if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Command accepted (result=%d)", result->result);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Command REJECTED (result=%d)", result->result);
          }
          // Shut down after one shot
          rclcpp::shutdown();
        });


      timer_->cancel();  // Don't fire again
    });
  }


private:
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoTestNode>());
  return 0;
}
