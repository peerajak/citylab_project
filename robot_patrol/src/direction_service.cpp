#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {

    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::moving_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_callback(const std::shared_ptr<GetDirection::Request> request,
                       const std::shared_ptr<GetDirection::Response> response) {
    RCLCPP_INFO(this->get_logger(), "laser frame id %s",
                request->laser_data.header.frame_id.c_str());
    // sensor_msgs::msg::LaserScan::SharedPtr msg
    //  sensor_msgs::msg::LaserScan msg;
    //  msg = request->laser_data;
    // RCLCPP_INFO(this->get_logger(), "laser data[%d]: %f", i,
    //           request->laser_data.ranges[i]);
    for (int i = 0; i < 10; i++) {
      RCLCPP_INFO(this->get_logger(), "laser data[%d]: %f", i,
                  request->laser_data.ranges[i]);
    }

    response->direction = "test";
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}