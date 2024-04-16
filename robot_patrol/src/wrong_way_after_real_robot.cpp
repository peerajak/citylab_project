/*
Applying
    4.12   Multiple Mutually Exclusive Callback Groups
    (Multiple  is multiple group)
to
    Checkpoint 5: citylab_project/robot_patrol.cpp
*/

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace std::chrono_literals;

#define pi 3.14

int scan_index_from_radian(float radian) {
  int indx;

  if (radian >= 0 && radian <= (pi / 2)) {
    indx = int((2 / pi) * 165 * radian);
  } else if (radian < 0 && radian >= -(pi / 2)) {
    indx = int((2 / pi) * 165 * radian + 659);
  }
  return indx;
};

float radian_from_scan_index(int scan_index) {
  float rad;
  if (659 >= scan_index && scan_index >= 495) {
    rad = float(scan_index - 659) / 165 * (pi / 2);
  } else if (164 >= scan_index && scan_index >= 0) {
    rad = float(scan_index) / 165 * (pi / 2);
  }
  return rad;
};

float degree_from_scan_index(int scan_index) {
  float deg;
  if (659 >= scan_index && scan_index >= 495) {
    deg = float(scan_index - 659) / 165 * 90;
  } else if (164 >= scan_index && scan_index >= 0) {
    deg = float(scan_index) / 165 * 90;
  }
  return deg;
};

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
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
    this->move_robot(ling);
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback End");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    float speed_x = 0.1, radian_max = 0, radian_min = 0, radian_select,
          value_select; // prev good value -0.1
    int index_select;
    float endanged_min = 0.5;
    const float angle_increment = 0.009529590606689453;
    float radian_avoid_gap = pi / 8, radian_check_ahead_gap = pi / 10;

    std::vector<std::tuple<float, int>> front_ranges;

    for (int i = 0; i < 165; i++) {
      if (msg->ranges[i] < 200 && msg->ranges[i] > 0) {
        front_ranges.push_back(std::tuple(msg->ranges[i], i));
      }
    }

    for (int i = 495; i < 660; i++) {
      if (msg->ranges[i] < 200 && msg->ranges[i] > 0) {
        front_ranges.push_back(std::tuple(msg->ranges[i], i));
      }
    }
    sort(front_ranges.begin(), front_ranges.end());

    RCLCPP_INFO(this->get_logger(), "sort max %d:%f,%f",
                std::get<1>(front_ranges.back()),
                std::get<0>(front_ranges.back()),
                degree_from_scan_index(std::get<1>(front_ranges.back())));
    RCLCPP_INFO(this->get_logger(), "sort min %d:%f,%f",
                std::get<1>(front_ranges[0]), std::get<0>(front_ranges[0]),
                degree_from_scan_index(std::get<1>(front_ranges[0])));

    int min_index = std::get<1>(front_ranges[0]);
    float min_value = std::get<0>(front_ranges[0]);

    int max_index = std::get<1>(front_ranges.back());
    float max_value = std::get<0>(front_ranges.back());

    // RCLCPP_INFO(this->get_logger(),
    //             "_direction %d:%f, %d:%f, %d:%f min at %d,%f max at %d,%f",
    //             0,
    //            msg->ranges[0], 360, msg->ranges[360], 659, msg->ranges[659],
    //            min_index, *it_min, max_index, *it_max);

    radian_max = radian_from_scan_index(max_index);
    radian_min = radian_from_scan_index(min_index);
    radian_select = radian_max;
    value_select = max_value;
    index_select = max_index;
    if (std::abs(radian_max - radian_min) < radian_avoid_gap) {

      auto ita = front_ranges.begin();
      do {
        if (std::get<0>(*ita) > endanged_min) {
          index_select = std::get<1>(*ita);
          radian_select = radian_from_scan_index(index_select);
          value_select = std::get<0>(*ita);
        }
        ita++;

      } while (std::abs(radian_select - radian_min) < radian_avoid_gap &&
               ita < front_ranges.end());
    }
    float checking_ahead_radian_min =
        radian_select - radian_check_ahead_gap / 2;
    float checking_ahead_radian_max =
        radian_select + radian_check_ahead_gap / 2;
    float checking_iter = checking_ahead_radian_min;
    float checking_value, found_value, found_radian;
    int checking_index;
    bool ahead_obstracle = false;
    while (checking_iter < checking_ahead_radian_max) {

      checking_index = scan_index_from_radian(checking_iter);

      for (auto it = front_ranges.begin(); it < front_ranges.end(); it++) {
        if (checking_index == std::get<1>(*it)) {

          checking_value = std::get<0>(*it);
          if (checking_value < endanged_min) {
            ahead_obstracle = true;
            found_value = checking_value;
            found_radian = checking_iter;
            RCLCPP_INFO(this->get_logger(),
                        "found radian  %f has obstrucle ahead %f meter",
                        checking_iter, checking_value);
          }
        }
      }

      checking_iter += angle_increment;
    }

    if (ahead_obstracle) {
      RCLCPP_INFO(this->get_logger(), "sharp turning");
      if (found_radian > 0) {
        radian_select = pi / 2;
      } else {
        radian_select = -pi / 2;
      }
    }
    RCLCPP_INFO(this->get_logger(), "radian select %f,%f", radian_select,
                value_select);

    ling.angular.z = 0.5 * radian_select;
    ling.linear.x = speed_x;
  }
  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  int direction_ = 360; //[0-719]
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
/*
angle_min: 0.0
angle_max: 6.28000020980835
angle_increment: 0.009529590606689453

Thus there are 6.28/0.00953 = 659 lines for 360 degree (6.28=2pi)
0,659 degree msg->range[0] start from x axis,
164 degree msg->range[164] means x +90degree,
495 degree msg->range[495] means x -90degree.

case 1. 0-164 has 165 lines of scan
0-164 is 0-pi/2
radian = (direction_/164)*(pi/2)

case 2 495-659 has 165 lines of scan
495-659 is -pi/2-0
radian = (direction-659)/164*pi/2

We will test this tomorrow
*/