#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include <cmath>
#include <geometry_msgs/msg/point.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

#define pi 3.14
/*
    This is the action server that subscribe to /odom topic and also move robot
    therefore, there are 3 main IOs
    1. subscribe to /odom topic with a subscriber
    2. publish to /cmd_vel topic with a publisher
    3. Action server.

    The callbacks
    1. subscriber needs 1 callback, odom_callback
    2. publisher needs 1 callback. need a timer to control its speed.
   server's callbacks.
    3. Action server needs 3 callbacks: handle_goal,
   handle_cancel,handle_accepted.

    Question: Do I need Callback groups?
    I think, Yes for subsciber only. because
    1) for a subscriber, do it
    2) Actopm server?
        After reading, I think Action servers need no callback group because
        1. There is no callback group option in the action_server.option
        2. execute() is being done in a new detached thread. Thus might now slow
            down the main thread.
*/

class GoToPoseActionServer : public rclcpp::Node {
public:
  using GoToPose = robot_patrol::action::GoToPose;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<GoToPose>;

  explicit GoToPoseActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_server", options) {
    using namespace std::placeholders;
    /*timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);*/
    odom1_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom1_callback_group_;
    subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&GoToPoseActionServer::odom_callback, this,
                  std::placeholders::_1),
        options1);

    this->action_server_ = rclcpp_action::create_server<GoToPose>(
        this, "go_to_pose",
        std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
        std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
        std::bind(&GoToPoseActionServer::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    /*timer1_ = this->create_wall_timer(
        100ms, std::bind(&GoToPoseActionServer::timer1_callback, this),
        timer_callback_group_);*/
  }

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  // rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::CallbackGroup::SharedPtr
      odom1_callback_group_; //, timer_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;
  geometry_msgs::msg::Twist ling;
  geometry_msgs::msg::Point desire_pos_, current_pos_;
  geometry_msgs::msg::Quaternion desire_angle_, current_angle_;
  double target_yaw_rad_, current_yaw_rad_;

  void move_robot(geometry_msgs::msg::Twist &msg) { publisher_->publish(msg); }
  bool check_reached_goal_angle(float delta_error = 0.3) {
    float delta_theta = std::abs(current_yaw_rad_ - target_yaw_rad_);
    return delta_theta < delta_error; // IN GOAL return true else false;
  }
  bool check_reached_goal_pos(const geometry_msgs::msg::Point goal,
                              const geometry_msgs::msg::Point current_pos,
                              float delta_error = 0.1) {

    // print_2Dposition(goal, "GOAL");
    // print_2Dposition(current_pos, "CURRENT_POS");

    float delta_x = std::abs(goal.x - current_pos.x);
    float delta_y = std::abs(goal.y - current_pos.y);
    bool result = false;
    RCLCPP_INFO(this->get_logger(),
                "[desire, current]=['%f','%f'] vs ['%f',%f]", goal.x, goal.y,
                current_pos.x, current_pos.y);

    if (delta_x <= delta_error) {
      if (delta_y <= delta_error) {
        RCLCPP_INFO(this->get_logger(), "IN GOAL[dx,dy,error]=['%f','%f','%f']",
                    delta_x, delta_y, delta_error);
        result = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "[DY,ERROR]=['%f','%f']", delta_y,
                    delta_error);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "[DX,ERROR]=['%f','%f']", delta_x,
                  delta_error);
    }

    return result;
  }

  double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }

  float theta_from_arctan(float x_target, float x_current, float y_target,
                          float y_current) {
    // float atan_ans = std::atan((y_target - y_current) / (x_target -
    // x_current));
    float ret;
    if (x_target < x_current && y_target < y_current)
      ret =
          -pi / 2 - std::atan((x_target - x_current) / (y_target - y_current));
    else if (x_target < x_current && y_target >= y_current)
      ret = pi / 2 - std::atan((x_target - x_current) / (y_target - y_current));
    else
      ret = std::atan((y_target - y_current) / (x_target - x_current));
    return ret;
  }

  /*timer callback
  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 Callback Start");
    this->move_robot(ling);
  }*/
  /* odom topic subscriber callback */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }

  /* GoToPose Message
  geometry_msgs/Pose2D goal_pos
  ---
  bool status
  ---
  geometry_msgs/Pose2D current_pos
  */
  /* sets of action server callback: 1. handle_goal*/
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose::Goal> goal) {

    // TODO
    //(void)uuid;
    desire_pos_.x = goal->goal_pos.x;
    desire_pos_.y = goal->goal_pos.y;
    target_yaw_rad_ = theta_from_arctan(desire_pos_.x, current_pos_.x,
                                        desire_pos_.y, current_pos_.y);
    RCLCPP_INFO(
        this->get_logger(),
        "Received goal desire_position at x:%f y:%f, calculate theta %f",
        goal->goal_pos.x, goal->goal_pos.y, target_yaw_rad_);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /* sets of action server callback: 2. handle_cabcel*/
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    //  TODO
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /* sets of action server callback: 3. handle_accept*/
  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPoseActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();

    // auto &message = feedback->current_pos;
    auto result = std::make_shared<GoToPose::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    while (!check_reached_goal_angle() && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->status = false; // message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      ling.angular.z = target_yaw_rad_ - current_yaw_rad_;
      if (std::abs(ling.angular.z ) >1)
           ling.angular.z *= 0.1;
      move_robot(ling);
      feedback->current_pos.x = current_pos_.x;
      feedback->current_pos.y = current_pos_.y;
      feedback->current_pos.theta = current_yaw_rad_;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),
                  "Publish feedback current pos=['%f','%f'] target rad "
                  "'%f',current rad %f, angular speed %f",
                  current_pos_.x, current_pos_.y, target_yaw_rad_,
                  current_yaw_rad_, ling.angular.z);
      loop_rate.sleep();
    }

    while (!check_reached_goal_pos(desire_pos_, current_pos_, 0.1) &&
           rclcpp::ok()) {
      // while (true && rclcpp::ok()) {
      //  Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false; // message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Move robot forward and send feedback
      // message = "Moving forward...";
      target_yaw_rad_ = theta_from_arctan(desire_pos_.x, current_pos_.x,
                                          desire_pos_.y, current_pos_.y);
      ling.linear.x = 0.1; // TODO move robot logic here
      ling.angular.z = target_yaw_rad_ - current_yaw_rad_;

      move_robot(ling);
      feedback->current_pos.x = current_pos_.x;
      feedback->current_pos.y = current_pos_.y;
      feedback->current_pos.theta = current_yaw_rad_;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),
                  "Publish feedback current pos=['%f','%f'] target rad "
                  "'%f',current rad %f, angular speed %f",
                  current_pos_.x, current_pos_.y, target_yaw_rad_,
                  current_yaw_rad_, ling.angular.z);

      loop_rate.sleep();
    }

    ling.linear.x = 0;
    ling.angular.z = 0;
    // Check if goal is done
    if (rclcpp::ok()) {

      result->status = true;
      ling.linear.x = 0;
      ling.angular.z = 0;
      move_robot(ling);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
}; // class GoToPoseActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPoseActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}