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
#include <signal.h>
#include <tuple>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace std::chrono_literals;

#define pi 3.14
#define policy_broadest false
const float angle_increment = 0.00953;
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

// step 0. class band (min_radian, max_radian), get mean_radian
// step 1. class band constructor for loop front_ranges and insert consecutive
// radian into a band step 2. class band zoo insert only large enough band, keep
// all bands in band_zoo class step 3. Choose largest band's mean radian.

class band {
private:
  float _min_allow;
  float _max_allow;
  int _size;
  std::vector<float> msg_radian;
  std::vector<float> msg_values;

public:
  enum insertable_state { insertable, full } _state;

  band(float min_allow, float max_allow)
      : _min_allow(min_allow), _max_allow(max_allow), _size(0),
        _state(band::insertable_state::insertable) {}

  int insert(float a_radian, float value) {
    float interested_value = std::min(value, _max_allow);
    if (interested_value > _min_allow) {
      if (msg_radian.size() == 0) { // firstly pushed
        msg_radian.push_back(a_radian);
        msg_values.push_back(interested_value);
        _size++;
        _state = band::insertable_state::insertable;
        return 0;
      }
      // check consecutivte index
      auto find_it = std::find_if(
          msg_radian.begin(), msg_radian.end(),
          [&a_radian](float r) { return std::abs(r - a_radian) < 0.1; });
      if (find_it < msg_radian.end()) {
        msg_radian.push_back(a_radian);
        msg_values.push_back(interested_value);
        _size++;
        _state = band::insertable_state::insertable;
        return 0;
      } else {
        _state = band::insertable_state::full;
        return 1;
      }

    } else {
      return 2;
    }
  }

  std::tuple<float, float> get_boundary() { // return min,max

    if (msg_radian.size() > 1)
      return std::tuple<float, float>(msg_radian[0], msg_radian.back());
    if (msg_radian.size() == 1)
      return std::tuple<float, float>(msg_radian[0], msg_radian[0]);

    return std::tuple<float, float>(0, 0);
  }

  float get_deepest_value() {
    auto it_max = std::max_element(msg_values.begin(), msg_values.end());
    return *it_max;
  }
  std::tuple<float, float> get_deepest_radian() {
    auto it_max = std::max_element(msg_values.begin(), msg_values.end());
    int deepest_radian = msg_radian[std::distance(msg_values.begin(), it_max)];

    return std::tuple<float, float>(deepest_radian, *it_max);
  }
  std::tuple<float, float> get_shallowest_radian() {
    auto it_min = std::min_element(msg_values.begin(), msg_values.end());
    int shallowest_radian =
        msg_radian[std::distance(msg_values.begin(), it_min)];

    return std::tuple<float, float>(shallowest_radian, *it_min);
  }

  std::tuple<float, float> get_broadest_mid_radian() {
    float broadest_radian = (msg_radian[0] + msg_radian.back()) / 2;
    auto find_it = std::find_if(msg_radian.begin(), msg_radian.end(),
                                [&broadest_radian](float r) {
                                  return std::abs(r - broadest_radian) < 0.001;
                                });
    return std::tuple<float, float>(broadest_radian, *find_it);
  }

  int get_size() { return _size; }
  unsigned int get_msg_radian_size() { return msg_radian.size(); }
  unsigned int get_msg_values_size() { return msg_values.size(); }

  float get_statistic_max() {
    return *std::max_element(msg_values.begin(), msg_values.end());
  }
  float get_statistic_min() {
    return *std::min_element(msg_values.begin(), msg_values.end());
  }
  float get_statistic_mean() {
    return float(std::accumulate(msg_values.begin(), msg_values.end(), 0)) /
           _size;
  }
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

  static void mySigintHandler(int sig) {
    // RCLCPP_DEBUG(this->get_logger(), "/rotate_robot service stopped");
    rclcpp::shutdown();
  }

private:
  float calculate_best_radian_from_min_max(
      float radian_max, float radian_min, float max_value,
      float radian_avoid_gap, float endanged_min,
      std::vector<std::tuple<float, int>> &front_ranges) {

    float radian_select = radian_max;
    float value_select = max_value;
    if (std::abs(radian_max - radian_min) < radian_avoid_gap) {

      auto ita = front_ranges.begin();
      do {
        if (std::get<0>(*ita) > endanged_min) {
          radian_select = radian_from_scan_index(std::get<1>(*ita));
          value_select = std::get<0>(*ita);
        }
        ita++;
        // RCLCPP_INFO(this->get_logger(), "selecting new angle");
      } while (std::abs(radian_select - radian_min) < radian_avoid_gap &&
               ita < front_ranges.end());
    }
    return radian_select;
  }
  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
    this->move_robot(ling);
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback End");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    float speed_x = 0.1, radian_max = 0, radian_min = 0, radian_select = 0,
          value_select; // prev good value -0.1
    int index_select;
    float interested_max = 5.0; // prev good value 20
    float tolerated_min =
        0.4; // This value helps avoid obstrucle, set it high >0.7
    float endanged_min = 0.35;
    std::vector<band> aggregation_of_bands;
    float radian_avoid_gap = pi / 3, radian_check_ahead_gap = pi / 3;
    int smallest_allowable_band = 80; //// at least 120
    // float wide_band_max = 1.5, wide_band_min = endanged_min;

    std::vector<std::tuple<float, int>> front_ranges;
    for (int i = 495; i < 660; i++) {
      // if (msg->ranges[i] < 200 && msg->ranges[i] > wide_band_min) {
      front_ranges.push_back(std::tuple<float, int>(msg->ranges[i], i));
      // }
    }
    for (int i = 0; i < 165; i++) {
      // if (msg->ranges[i] < 200 && msg->ranges[i] > wide_band_min) {
      front_ranges.push_back(std::tuple<float, int>(msg->ranges[i], i));
      //}
    }

    auto ita = front_ranges.begin();
    std::shared_ptr<band> b_band(new band(tolerated_min, interested_max));

    while (ita < front_ranges.end()) {

      while (ita < front_ranges.end() &&
             b_band->_state == band::insertable_state::insertable) {
        float inserting_radian = radian_from_scan_index(std::get<1>(*ita));
        RCLCPP_DEBUG(this->get_logger(), "inserting radian %f:value %f",
                     inserting_radian, std::get<0>(*ita));
        int insert_result = b_band->insert(inserting_radian, std::get<0>(*ita));
        if (insert_result == 0) {
          RCLCPP_DEBUG(this->get_logger(), "inserted");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "NOT inserted %d", insert_result);
        }
        ita++;
      }

      std::tuple<float, float> boundary_aband = b_band->get_boundary();
      RCLCPP_DEBUG(this->get_logger(),
                   "inserted a band (%f,%f) size%d with max:value %f",
                   std::get<0>(boundary_aband), std::get<1>(boundary_aband),
                   b_band->get_size(), b_band->get_statistic_max());

      if (b_band->get_size() >= smallest_allowable_band) {
        aggregation_of_bands.push_back(*b_band);
      }

      if (b_band->_state == band::insertable_state::full) {
        b_band = std::make_shared<band>(tolerated_min, interested_max);
      }
    }

    // find biggest band

#if policy_broadest
    RCLCPP_INFO(this->get_logger(),
                "aggregation_of_bands size  %d finding broadest",
                aggregation_of_bands.size());
    auto result_it = std::max_element(
        aggregation_of_bands.begin(), aggregation_of_bands.end(),
        [](band a, band b) { return a.get_size() < b.get_size(); });

#else
    RCLCPP_INFO(this->get_logger(),
                "aggregation_of_bands size  %d finding deepest",
                aggregation_of_bands.size());
    auto result_it =
        std::max_element(aggregation_of_bands.begin(),
                         aggregation_of_bands.end(), [](band a, band b) {
                           return a.get_deepest_value() < b.get_deepest_value();
                         });
#endif
    if (result_it != aggregation_of_bands.end()) {
      band &c_band = *result_it;
      std::tuple<float, float> boundary_aband = c_band.get_boundary();
      RCLCPP_INFO(this->get_logger(), "found (%f,%f) size%d with max:value %f",
                  std::get<0>(boundary_aband), std::get<1>(boundary_aband),
                  c_band.get_size(), c_band.get_statistic_max());

      std::tuple<float, float> result_tuple = c_band.get_broadest_mid_radian();
      radian_select = std::get<0>(result_tuple);
      value_select = std::get<1>(result_tuple);
#if 0
      std::tuple<float, float> deepest_tuple = c_band.get_deepest_radian();
      radian_max = std::get<0>(deepest_tuple);
      int max_value = std::get<1>(deepest_tuple);
      std::tuple<float, float> shallowest_tuple =
          c_band.get_shallowest_radian();
      radian_min = std::get<0>(shallowest_tuple);
      // int min_value= std::get<1>(shallowest_tuple);
      radian_select = calculate_best_radian_from_min_max(
          radian_max, radian_min, max_value, radian_avoid_gap, endanged_min,
          front_ranges);
#endif
    }
#if 0
    else {
      int min_index = std::get<1>(front_ranges[0]);
      float min_value = std::get<0>(front_ranges[0]);

      int max_index = std::get<1>(front_ranges.back());
      float max_value = std::get<0>(front_ranges.back());

      // RCLCPP_INFO(this->get_logger(),
      //             "_direction %d:%f, %d:%f, %d:%f min at %d,%f max at %d,%f",
      //             0,
      //            msg->ranges[0], 360, msg->ranges[360], 659,
      //            msg->ranges[659], min_index, *it_min, max_index, *it_max);

      radian_max = radian_from_scan_index(max_index);
      radian_min = radian_from_scan_index(min_index);
      radian_select = calculate_best_radian_from_min_max(
          radian_max, radian_min, max_value, radian_avoid_gap, endanged_min,
          front_ranges);
    }
#endif
    /*
    if (value_select < endanged_min) {
      RCLCPP_INFO(this->get_logger(), "sharp turning");
      radian_select = pi / 2;
    }*/
    // step 0. class band (min_radian, max_radian), get mean_radian
    // step 1. class band constructor for loop front_ranges and insert
    // consecutive radian into a band step 2. class band zoo insert only large
    // enough band, keep all bands in band_zoo class step 3. Choose largest
    // band's mean radian.

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
  signal(SIGINT, Patrol::mySigintHandler);
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