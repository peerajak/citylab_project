#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <cassert>
#include <memory>
#include <algorithm>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;
#define pi 3.14
/* This is a service server */
const float angle_increment = 0.00953;

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

My understanding is direction x = 0 degree. CCW is positive, CW is
negative. Thus, left hand side of the robot is +90 degree, and right hand
side of the robot is -90 degree
*/

int scan_index_from_radian(float radian) {
  int indx;

  if (radian >= 0 && radian <= (pi / 2)) {
    indx = int((2 / pi) * 165 * radian);
  } else if (radian < 0 && radian >= -(pi / 2)) {
    indx = int((2 / pi) * 165 * radian + 659);
  }
  return indx;
};

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {

    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::service_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void
  service_callback(const std::shared_ptr<GetDirection::Request> request,
                   const std::shared_ptr<GetDirection::Response> response) {
    RCLCPP_INFO(this->get_logger(), "laser frame id %s",
                request->laser_data.header.frame_id.c_str());
    float max_interest_far = 0.7;
    /*
        float left_radian_threshold = pi / 6;
        float right_radian_threshold = -pi / 6;
        const int zero_right = 659;
        const int zero_left = 0;
        const int far_left_index = 164;  // scan_index_from_radian(pi/2);
        const int far_right_index = 495; // scan_index_from_radian(-pi/2);
        int left_index_threshold =
            scan_index_from_radian(left_radian_threshold) + 1; // 55
        int right_index_threshold =
            scan_index_from_radian(right_radian_threshold); // 604

        // My understanding is direction x = 0 degree. CCW is positive, CW is
        // negative. Thus, left hand side of the robot is +90 degree, and right
       hand
        // side of the robot is -90 degree

        auto it_far_right =
            std::next(request->laser_data.ranges.begin(), far_right_index);
        auto it_far_left =
            std::next(request->laser_data.ranges.begin(), far_left_index);
        auto it_left =
            std::next(request->laser_data.ranges.begin(), left_index_threshold);
        auto it_right =
            std::next(request->laser_data.ranges.begin(),
       right_index_threshold); auto it_0_left =
       std::next(request->laser_data.ranges.begin(), zero_left); auto it_0_right
       = std::next(request->laser_data.ranges.begin(), zero_right); RCLCPP_INFO(
            this->get_logger(),
            "num left index %d, num mid index %d ,num right index %d",
            int(std::distance(it_left, it_far_left)),
            int(std::distance(
                it_0_left, it_left)) + int(std::distance(it_right, it_0_right)),
            int(std::distance(it_far_right, it_right)));


        float sum_right =
            accumulate(it_far_right, it_right, 0);            // 495-604 =110
       lines float sum_left = accumulate(it_left, it_far_left, 0); // 55-164 =
       110 lines float sum_mid = accumulate(it_0_left, it_left, 0) +
            accumulate(it_right, it_0_right, 0); // 0-55 + 604-659 = 112 lines
                    */

    float sum_left = 0, sum_mid = 0, sum_right = 0;

    for (int i = 0; i <= 54; i++) {
      sum_mid += std::min(request->laser_data.ranges[i], max_interest_far);
    }
    for (int i = 605; i <= 659; i++) {
      sum_mid += std::min(request->laser_data.ranges[i], max_interest_far);
    }
    for (int i = 55; i <= 164; i++) {
      sum_left += std::min(request->laser_data.ranges[i], max_interest_far);
    }
    for (int i = 495; i <= 604; i++) {
      sum_right += std::min(request->laser_data.ranges[i], max_interest_far);
    }

    if (sum_left > sum_mid && sum_left > sum_right) {
      response->direction = "left";
    } else if (sum_mid > sum_left && sum_mid > sum_right) {
      response->direction = "forward";
    } else if (sum_right > sum_mid && sum_right > sum_left) {
      response->direction = "right";
    } else {
      assert(true);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
