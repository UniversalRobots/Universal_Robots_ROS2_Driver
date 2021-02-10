#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

using namespace std::chrono_literals;

#define INCREMENT 0.1  // rads

std::vector<std::string> name_order = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

class MinimalDriverTest : public rclcpp::Node
{
public:
  MinimalDriverTest() : Node("test_driver"), js_received_(false)
  {
    publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>("forward_command_controller_position/commands", 10);
    timer_ = this->create_wall_timer(1500ms, std::bind(&MinimalDriverTest::timer_callback, this));

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&MinimalDriverTest::topic_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
    if (!js_received_)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Joint states not received");
      return;
    }

    if (msg_internal_.position.size() != 6)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Joint states and command size mismatch");
      return;
    }

    auto message = std_msgs::msg::Float64MultiArray();
    message.data.resize(6);
    // set up dimensions
    message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    message.layout.dim[0].size = 6;
    message.layout.dim[0].stride = 1;
    message.layout.dim[0].label = "cmd";

    for (size_t i = 0; i < name_order.size(); ++i)
    {
      for (size_t j = 0; j < msg_internal_.name.size(); ++j)
      {
        if (msg_internal_.name[j] == name_order[i])
          message.data[i] = msg_internal_.position[j] + INCREMENT;
      }
    }

    publisher_->publish(message);
  }

  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (js_received_)
      return;

    js_received_ = true;
    msg_internal_ = *msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  bool js_received_;
  sensor_msgs::msg::JointState msg_internal_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalDriverTest>());
  rclcpp::shutdown();
  return 0;
}