#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class Cv3Node : public rclcpp::Node
{
public:
  Cv3Node(): Node("Cv3Node"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Cv3Node::timer_callback, this));
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10,
      std::bind(&Cv3Node::battery_callback, this, std::placeholders::_1));

    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
    this->declare_parameter("min_voltage", 32.0f);
    this->declare_parameter("max_voltage", 42.0f);
  }
private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Cv3Node";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float min_v = this->get_parameter("min_voltage").as_double();
    float max_v = this->get_parameter("max_voltage").as_double();
    float percentage = (msg->data - min_v) / (max_v - min_v) * 100.0f;
    auto out = std_msgs::msg::Float32();
    out.data = percentage;
    battery_pub_->publish(out);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  size_t count_;
};


int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<Cv3Node>());
rclcpp::shutdown();
return 0;
}