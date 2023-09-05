#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "neato2_interfaces/msg/bump.hpp"
#include <atomic>
#include <chrono>

/// The Node that encapsulates our main functionality 
class EmergencyStop : public rclcpp::Node {
  public:
    EmergencyStop()
    : Node("estop")
    {
      publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      bumper_subscription = this->create_subscription<neato2_interfaces::msg::Bump>(
          "bump",
          10,
          std::bind(
              &EmergencyStop::handle_bump,
              this,
              std::placeholders::_1));
      timer = rclcpp::create_timer(this,
                                   this->get_clock(),
                                   rclcpp::Duration::from_seconds(0.1),
                                   std::bind(&EmergencyStop::run_loop, this));
    }

    void handle_bump(const neato2_interfaces::msg::Bump::SharedPtr msg)
    {
      bumper_active = (msg->left_front == 1 ||
                       msg->right_front == 1 ||
                       msg->left_side == 1 ||
                       msg->right_side == 1);
    }

    void run_loop()
    {
      geometry_msgs::msg::Twist msg;
      if (bumper_active) {
        msg.linear.x = 0.0;
      } else {
        msg.linear.x = 0.1;
      }
      publisher->publish(msg);
    }

  private:
    rclcpp::Subscription<neato2_interfaces::msg::Bump>::SharedPtr bumper_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    std::atomic<bool> bumper_active;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmergencyStop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}