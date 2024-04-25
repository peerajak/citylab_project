#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
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
#include <cmath>

#define pi 3.14

int action_counter = 0;

double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
  double roll_rad, pitch_rad, yaw_rad;
  tf2::Quaternion odom_quat(qx, qy, qz, qw);
  tf2::Matrix3x3 matrix_tf(odom_quat);
  matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
  return yaw_rad; // In radian
};

float theta_from_arctan(float x_target, float x_current, float y_target,
                        float y_current) {
  // float atan_ans = std::atan((y_target - y_current) / (x_target -
  // x_current));
  float ret;
  if (x_target < x_current && y_target < y_current)
    ret = -pi / 2 - std::atan((x_target - x_current) / (y_target - y_current));
  else if (x_target < x_current && y_target >= y_current)
    ret = pi / 2 - std::atan((x_target - x_current) / (y_target - y_current));
  else
    ret = std::atan((y_target - y_current) / (x_target - x_current));
  return ret;
}

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
        std::chrono::milliseconds(0),
        std::bind(&GoToPoseActionClient::send_goal, this));

    start_position.x = -0.068263;
    start_position.y = 0.50008;
    start_position.theta = 0;

    current_pos_ = start_position;

    corner_bottom_left.x = 0.737;
    corner_bottom_left.y = 0.576;
    corner_bottom_left.theta = -pi / 2;

    corner_bottom_right.x = 0.88435;
    corner_bottom_right.y = -0.57935;
    corner_bottom_right.theta = -pi / 2;

    corner_top_left.x = -0.53170;
    corner_top_left.y = -0.47636;
    corner_top_left.theta = -pi / 2;

    corner_top_right.x = -0.55024;
    corner_top_right.y = 0.381047;
    corner_top_right.theta = -pi / 2;

    corner_goal_pose2d.push_back(corner_bottom_left);
    corner_goal_pose2d.push_back(corner_bottom_right);
    corner_goal_pose2d.push_back(corner_top_left);
    corner_goal_pose2d.push_back(corner_top_right);
    for (int i = 0; i < action_counter; i++) {
      std::rotate(corner_goal_pose2d.begin(), corner_goal_pose2d.begin() + 1,
                  corner_goal_pose2d.end());
    }
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

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
    goal_msg.goal_pos.theta =
        theta_from_arctan(goal_msg.goal_pos.x, current_pos_.x,
                          goal_msg.goal_pos.y, current_pos_.y);

    RCLCPP_INFO(this->get_logger(), "Sending goal x:%f y:%f",
                goal_msg.goal_pos.x, goal_msg.goal_pos.y);

    auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&GoToPoseActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&GoToPoseActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&GoToPoseActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  std::vector<geometry_msgs::msg::Pose2D> corner_goal_pose2d;
  geometry_msgs::msg::Pose2D start_position;
  geometry_msgs::msg::Pose2D corner_bottom_right;
  geometry_msgs::msg::Pose2D corner_bottom_left;
  geometry_msgs::msg::Pose2D corner_top_left;
  geometry_msgs::msg::Pose2D corner_top_right;
  geometry_msgs::msg::Pose2D current_pos_;

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

    RCLCPP_INFO(this->get_logger(), "Feedback received: x %f, y %f, theta %f",
                feedback->current_pos.x, feedback->current_pos.y,
                feedback->current_pos.theta);
    current_pos_.x = feedback->current_pos.x;
    current_pos_.y = feedback->current_pos.y;
    current_pos_.theta = feedback->current_pos.theta;
  }

  void result_callback(const GoalHandlePose::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {

    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_ERROR(this->get_logger(), "Goal was success? %s",
                   result.result->status ? "yes" : "no");

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

  rclcpp::executors::MultiThreadedExecutor executor;

  while (rclcpp::ok()) {
    auto action_client = std::make_shared<GoToPoseActionClient>();
    executor.add_node(action_client);
    while (!action_client->is_goal_done()) {
      executor.spin_some();
    }
    executor.remove_node(action_client);
    action_counter++;
  }
  rclcpp::shutdown();
  return 0;
}