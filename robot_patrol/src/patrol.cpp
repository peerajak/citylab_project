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

class band {
private:
  float _min_allow;
  float _max_allow;
  int _size;
  std::vector<int> msg_index;
  std::vector<float> msg_values;

public:
  enum insertable_state { insertable, full } _state;
  band(float min_allow, float max_allow)
      : _min_allow(min_allow), _max_allow(max_allow), _size(0),
        _state(band::insertable_state::insertable) {}

  bool insert(int a_msg_index, float value) {
    float interested_value = std::min(value, _max_allow);
    if (interested_value > _min_allow) {
      if (msg_index.size() == 0) { // firstly pushed
        msg_index.push_back(a_msg_index);
        msg_values.push_back(interested_value);
        _size++;
        _state = band::insertable_state::insertable;
        return true;
      }
      // check consecutivte index
      if (find(msg_index.begin(), msg_index.end(), a_msg_index - 1) !=
          msg_index.end()) {

        msg_index.push_back(a_msg_index);
        msg_values.push_back(interested_value);
        _size++;
        _state = band::insertable_state::insertable;
        return true;
      } else {
        _state = band::insertable_state::full;
        return false;
      }

    } else {
      return false;
    }
  }
  std::tuple<int, int> get_boundary() {

    if (msg_index.size() > 1)
      return std::tuple<int, int>(msg_index[0], msg_index.back());
    if (msg_index.size() == 1)
      return std::tuple<int, int>(msg_index[0], msg_index[0]);

    return std::tuple<int, int>(0, 0);
  }

  int get_direction() {
    auto it_max = std::max_element(msg_values.begin(), msg_values.end());
    int deepest_index = msg_index[std::distance(msg_values.begin(), it_max)];

    return deepest_index;
  }

  int get_direction(float threshold_min) {
    auto it_max = std::max_element(msg_values.begin(), msg_values.end());
    int deepest_index = msg_index[std::distance(msg_values.begin(), it_max)];
    int broadest_index = int((msg_index[0] + msg_index.back()) / 2);

    if (this->get_statistic_mean() < threshold_min) {
      return broadest_index;
    }
    return deepest_index;
  }

  float get_value_from_index(int index) {
    auto it = find(msg_index.begin(), msg_index.end(), index);

    if (it != msg_index.end()) {

      int i = it - msg_index.begin();
      return msg_values[i];
    } else {
      // If the element is not
      // present in the vector
      return -1;
    }
  }
  int get_size() { return _size; }
  unsigned int get_msg_index_size() { return msg_index.size(); }
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

private:
  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
    this->move_robot(ling);
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback End");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Laser Callback Start");
    float speed_x = 0.1;       // prev good value -0.1
    float interested_max = 20; // prev good value 20
    float tolerated_min =
        0.7; // This value helps avoid obstrucle, set it high >0.7
    float endanged_min = 0.36;         // prev good value 0.36
    float through_threshold = 0.5;     // prv good value 0.4
    int smallest_allowable_band = 120; //// prev good value 100
    int avoid_gap = 350;               // prev good value 450
    int turn_around_gap = 250;         // not more than 250
    // bool policy_broadest = false; // ture=broadest band, false =farrest band
    std::vector<band> aggregation_of_bands;
    int raw_direction, raw_avoid_direction;
    float raw_largest_value, raw_smallest_value, band_value;
    int band_direction = -360;

    auto it_max = std::max_element(msg->ranges.begin(), msg->ranges.end());
    raw_direction = std::distance(msg->ranges.begin(), it_max);
    raw_largest_value = *it_max;

    auto it_min = std::min_element(msg->ranges.begin(), msg->ranges.end());
    raw_smallest_value = *it_min;
    raw_avoid_direction = std::distance(msg->ranges.begin(), it_min);

    auto ita = msg->ranges.begin();
    std::shared_ptr<band> b_band(new band(tolerated_min, interested_max));
    int counterb = 0;
    while (ita < msg->ranges.end()) {

      while (ita < msg->ranges.end() &&
             b_band->_state == band::insertable_state::insertable) {

        RCLCPP_DEBUG(this->get_logger(), "inserting index %d:value %f",
                     counterb, *ita);
        if (b_band->insert(counterb, *ita)) {
          RCLCPP_DEBUG(this->get_logger(), "inserted");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "NOT inserted");
        }

        counterb++;
        ita++;
      }

      std::tuple<int, int> boundary_aband = b_band->get_boundary();
      RCLCPP_DEBUG(this->get_logger(),
                   "inserted a band (%d,%d)with max:value %f",
                   std::get<0>(boundary_aband), std::get<1>(boundary_aband),
                   b_band->get_statistic_max());

      if (b_band->get_size() >= smallest_allowable_band &&
          b_band->get_statistic_max() >= through_threshold) {
        aggregation_of_bands.push_back(*b_band);
      }

      if (b_band->_state == band::insertable_state::full) {
        b_band = std::make_shared<band>(tolerated_min, interested_max);
      }
    }
    float max_maxfar = 0;
    for (auto itaa = aggregation_of_bands.begin();
         itaa < aggregation_of_bands.end(); itaa++) {
      band &c_band = (*itaa);
      float cur_maxfar = c_band.get_statistic_min();
      std::tuple<int, int> boundary_aband = c_band.get_boundary();
      RCLCPP_INFO(this->get_logger(),
                  "found a band (%d,%d)with max:value %f, min:value%f",
                  std::get<0>(boundary_aband), std::get<1>(boundary_aband),
                  cur_maxfar, c_band.get_statistic_min());
      if (cur_maxfar > max_maxfar) {
        max_maxfar = cur_maxfar;

        // band_direction = c_band.get_direction(tolerated_min + 0.2);
        band_direction = c_band.get_direction();
        band_value = c_band.get_value_from_index(band_direction);
        RCLCPP_INFO(this->get_logger(),
                    "maxing a band (%d,%d)with min:value %f",
                    std::get<0>(boundary_aband), std::get<1>(boundary_aband),
                    cur_maxfar);
      }
    }
    RCLCPP_INFO(this->get_logger(), "end_found_a band");
    RCLCPP_INFO(this->get_logger(),
                "band_direction %d - raw_avoid_direction%d) , avoid_gap%d",
                band_direction, raw_avoid_direction, avoid_gap);
    RCLCPP_INFO(this->get_logger(), "band_value %f <= endanged_min %f",
                band_value, endanged_min);
    if (band_value <= endanged_min) {
      if (std::abs(band_direction - raw_avoid_direction) <= avoid_gap) {

        if (std::abs(720 - raw_avoid_direction) <= turn_around_gap) {
          direction_ = 720 - (raw_avoid_direction +
                              int((endanged_min - raw_smallest_value) * 1000));
          RCLCPP_INFO(this->get_logger(),
                      "turning! collison ahead.  raw_direction %d, "
                      "raw_avoid_direction %d, direction_ %d",
                      raw_direction, raw_avoid_direction, direction_);
        } else {
          RCLCPP_INFO(this->get_logger(), "tightly turned!");
          if ((720 - raw_avoid_direction) < raw_avoid_direction) {

            direction_ = 0;
          } else {
            direction_ = 720;
          }
        }

        RCLCPP_INFO(this->get_logger(), "collision ahead avoid %d, goto %d",
                    raw_avoid_direction, direction_);

      } else {
        RCLCPP_INFO(this->get_logger(),
                    "direct collision ahead tightly turned!");
        if ((720 - raw_avoid_direction) < raw_avoid_direction) {
          direction_ = 0;
        } else {
          direction_ = 720;
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "through road ahead");
      if (band_direction >= 0) {
        direction_ = band_direction;

      } else {
        direction_ = raw_direction;
        RCLCPP_INFO(this->get_logger(), "no band found, use raw direction");
      }
    }
    ling.linear.x = speed_x;
    ling.angular.z = -((float(direction_) / 4) / 180 * 3.14 - (3.14 / 2)) * 0.5;
    RCLCPP_INFO(this->get_logger(),
                "_direction %d, raw_direction %d:%f "
                ",angular velocity z %f",
                direction_, raw_direction, raw_largest_value, ling.angular.z);
    /*
    int rangesize = 719;
    int midrange = rangesize / 2, left_onethird = rangesize / 3,
        right_onethird = rangesize * 2 / 3;
    if (msg->ranges[midrange] <= endanged_min ||
        msg->ranges[left_onethird] <= endanged_min ||
        msg->ranges[right_onethird] <= endanged_min) {
      RCLCPP_INFO(this->get_logger(), "collision ahead %d/%d:%f", midrange,
                  rangesize, msg->ranges[midrange]);
      ling.linear.x = 0.0;
      ling.linear.y += 0.01;
      ling.angular.z = 1;
    } else {
      RCLCPP_INFO(this->get_logger(), "%d/%d:%f", midrange, rangesize,
                  msg->ranges[midrange]);
      ling.linear.x = 0.03;
      ling.angular.z = 0;
    }
     */
    /*if (raw_smallest_value <= endanged_min) {
      RCLCPP_INFO(this->get_logger(), "collision ahead ");
      ling.linear.x = -speed_x;
      ling.angular.z = -ling.angular.z;
    }*/
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