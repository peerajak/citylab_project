#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include <geometry_msgs/msg/point.h>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <vector>

//#include "geometry_msgs/Point.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define pi 3.14

class GoToPoseActionClient : public rclcpp::Node {
public:
  using GoToPose = robot_patrol::action::GoToPose;
  using GoalHandlePose = rclcpp_action::ClientGoalHandle<GoToPose>;

  explicit GoToPoseActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<GoToPose>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "go_to_pose");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20000),
        std::bind(&GoToPoseActionClient::send_goal, this));
    corner_bottom_right.x = 0.5;
    corner_bottom_right.y = 0.5;
    corner_bottom_right.theta = pi / 2;

    corner_bottom_left.x = 0.2;
    corner_bottom_left.y = 0.5;
    corner_bottom_left.theta = pi / 2;

    corner_top_left.x = 0.2;
    corner_top_left.y = 0.2;
    corner_top_left.theta = pi / 2;

    corner_top_right.x = 0.2;
    corner_top_right.y = 0.5;
    corner_top_right.theta = pi / 2;

    corner_goal_pose2d.push_back(corner_bottom_right);
    corner_goal_pose2d.push_back(corner_bottom_left);
    corner_goal_pose2d.push_back(corner_top_left);
    corner_goal_pose2d.push_back(corner_top_right);
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    // this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = GoToPose::Goal();

    goal_msg.goal_pos.x = corner_goal_pose2d[0].x;
    goal_msg.goal_pos.y = corner_goal_pose2d[0].y;
    goal_msg.goal_pos.theta = corner_goal_pose2d[0].theta;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&GoToPoseActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&GoToPoseActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&GoToPoseActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::rotate(corner_goal_pose2d.begin(), corner_goal_pose2d.begin() + 1,
                corner_goal_pose2d.end());
  }

private:
  rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  std::vector<geometry_msgs::msg::Pose2D> corner_goal_pose2d;
  geometry_msgs::msg::Pose2D corner_bottom_right; //{1., 1., pi / 2};
  geometry_msgs::msg::Pose2D corner_bottom_left;  //(0, 1, pi / 2);
  geometry_msgs::msg::Pose2D corner_top_left;     //(0, 0, pi / 2);
  geometry_msgs::msg::Pose2D corner_top_right;    //(0, 1, pi / 2);

  void goal_response_callback(const GoalHandlePose::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandlePose::SharedPtr,
      const std::shared_ptr<const GoalHandlePose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback received:");
  }

  void result_callback(const GoalHandlePose::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received:");
  }
}; // class GoToPoseActionClient

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GoToPoseActionClient>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}