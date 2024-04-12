/*
Applying
    4.12   Multiple Mutually Exclusive Callback Groups
    (Multiple means multiple group)
to
    Checkpoint 5: citylab_project/robot_patrol.cpp
*/

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node") {

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_2;
    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
        options1);

    timer1_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer1_callback, this), callback_group_1);

    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback Start");
    this->move_robot(ling);
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback End");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float near_collision = 0.25;
    float current_max = 0;
    int current_max_index;
    for (int i = 0; i < max_direction_; i++) {
      if (msg->ranges[i] > current_max) {
        current_max = msg->ranges[i];
        current_max_index = i;
      } else if (msg->ranges[i] == current_max) {
        if (std::abs(i - max_direction_ / 2) <
            std::abs(current_max_index - max_direction_ / 2)) {
          current_max = msg->ranges[i];
          current_max_index = i;
        }
      }
    }
    direction_ = current_max_index;
    // direction_ = *std::max_element(msg->ranges.begin(),msg->ranges.end());
    ling.linear.x = -0.1;
    ling.angular.z = (float(direction_) / 4 - float(max_direction_) / 8) / 90 *
                     3.14 / 2 * 0.3;
    RCLCPP_INFO(this->get_logger(), "_direction %d:angular velocity z %f",
                direction_, ling.angular.z);
    /*
    int midrange = rangesize / 2, left_onethird =
    rangesize / 3, right_onethird = rangesize * 2
    / 3; if (msg->ranges[midrange] <=
    near_collision || msg->ranges[left_onethird]
    <= near_collision) {
      RCLCPP_INFO(this->get_logger(), "collision
    ahead %d/%d:%f", midrange, rangesize,
    msg->ranges[midrange]); ling.linear.x =
    -0.01; ling.angular.z = 0.7; } else if
    (msg->ranges[right_onethird] <=
    near_collision) { ling.linear.x = -0.01;
      ling.angular.z = -0.7;
    } else {
      RCLCPP_INFO(this->get_logger(), "%d/%d:%f",
    midrange, rangesize, msg->ranges[midrange]);
      ling.linear.x = -0.1;
      ling.angular.z = 0;
    }
    */
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  int direction_; //[0-719]
  const int max_direction_ = 719;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the Node
  // float sleep_time1 = 1.0;

  std::shared_ptr<Patrol> laser_timer_node = std::make_shared<Patrol>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(laser_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}