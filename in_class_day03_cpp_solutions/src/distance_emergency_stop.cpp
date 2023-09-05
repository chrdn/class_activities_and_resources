#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <atomic>
#include <chrono>

/// The Node that encapsulates our main functionality 
class EmergencyStop : public rclcpp::Node {
  public:
    EmergencyStop()
    : Node("estop"), Kp(0.4), target_distance(0.5), distance_to_obstacle(-1.0) 
    {
      publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan",
          10,
          std::bind(
              &EmergencyStop::handle_scan,
              this,
              std::placeholders::_1));
      timer = rclcpp::create_timer(this,
                                   this->get_clock(),
                                   rclcpp::Duration::from_seconds(0.1),
                                   std::bind(&EmergencyStop::run_loop, this));
    }

    void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      if (msg->ranges[0] != 0.0) {
        distance_to_obstacle = msg->ranges[0];
      }
    }

    void run_loop()
    {
      geometry_msgs::msg::Twist msg;
      
      if (distance_to_obstacle < 0.0) {
        msg.linear.x = 0.1;
      } else {
        msg.linear.x = Kp * (distance_to_obstacle - target_distance);
      }
      publisher->publish(msg);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    const float Kp;
    const float target_distance;
    std::atomic<float> distance_to_obstacle;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmergencyStop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}