#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DummyTwistPub : public rclcpp::Node
{
public:
  DummyTwistPub() : Node("dummy_twist_pub")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&DummyTwistPub::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5;
    message.angular.z = 0.1;
    RCLCPP_INFO(this->get_logger(), "Publishing: linear=%f, angular=%f", message.linear.x, message.angular.z);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyTwistPub>());
  rclcpp::shutdown();
  return 0;
}