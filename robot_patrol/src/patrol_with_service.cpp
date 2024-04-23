/*
Applying
    4.12   Multiple Mutually Exclusive Callback Groups
    (Multiple  is multiple group)
to
    Checkpoint 5: citylab_project/robot_patrol.cpp
*/

#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"
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
using GetDirection = robot_patrol::srv::GetDirection;
/* This is a service subscriber */


class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node") {
    client_ = this->create_client<GetDirection>("direction_service");

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
  /* timer1 callback */
  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }
    if (last_laser_ != nullptr) {
      RCLCPP_DEBUG(this->get_logger(), "sending request..");
      auto request = std::make_shared<GetDirection::Request>();
      request->laser_data = *last_laser_;

      service_done_ = false;
      auto result_future = client_->async_send_request(
          request,
          std::bind(&Patrol::response_callback, this, std::placeholders::_1));
    } else {
      RCLCPP_DEBUG(this->get_logger(), "NOT sending request..");
    }

    this->move_robot(ling);
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback End");
  }
  /* laser callback*/
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Laser Callback Start");
    last_laser_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
  }
  /*respond from service server callback*/
  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto service_response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s",
                  service_response->direction.c_str());
      std::string direction_from_service(service_response->direction.c_str());
      if (direction_from_service == "left") {
        ling.angular.z = 0.5;
        ling.linear.x = 0.1;
      } else if (direction_from_service == "forward") {
        ling.angular.z = 0.0;
        ling.linear.x = 0.1;
      } else { // direction_from_service == "right"
        ling.angular.z = -0.5;
        ling.linear.x = 0.1;
      }
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  void move_robot(geometry_msgs::msg::Twist &msg) { publisher1_->publish(msg); }
  bool is_service_done() const { return this->service_done_; }
  geometry_msgs::msg::Twist ling;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  rclcpp::Client<GetDirection>::SharedPtr client_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> last_laser_ = nullptr;
  bool service_done_ = false;
  int direction_;
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

*/