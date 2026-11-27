#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class GotoGoalNode : public rclcpp::Node {
public:
  GotoGoalNode() : Node("goto_goal_node") {
    kp_lin_           = this->declare_parameter<double>("kp_lin", 2.5);
    kp_yaw_           = this->declare_parameter<double>("kp_yaw", 1.8);
    max_lin_vel_      = this->declare_parameter<double>("max_lin_vel", 0.75);  // m/s
    max_yaw_vel_      = this->declare_parameter<double>("max_yaw_vel", 1.00);  // rad/s
    pos_tol_          = this->declare_parameter<double>("pos_tol", 0.03);      // m (3 cm)
    yaw_tol_          = this->declare_parameter<double>("yaw_tol", 0.06);      // rad (~3.4 deg)
    goal_alpha_       = this->declare_parameter<double>("goal_alpha", 0.6);    // live re-target blend
    auto_return_home_ = this->declare_parameter<bool>("auto_return_home", false);
    return_delay_s_   = this->declare_parameter<double>("return_delay_s", 2.5);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", 10,
      std::bind(&GotoGoalNode::odom_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/target_point", 10,
      std::bind(&GotoGoalNode::goal_callback, this, std::placeholders::_1));

    // 50 Hz control loop (matches /wheel_odom and Teensy 50 Hz PID)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&GotoGoalNode::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "goto_goal_node ready (50 Hz) | max_v=%.2f m/s, kp=%.2f, auto_return=%s",
      max_lin_vel_, kp_lin_, auto_return_home_ ? "true" : "false");
  }

private:
  static double wrap_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_received_ = true;
  }

  void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    const double nx = msg->x;
    const double ny = msg->y;
    const bool was_active = active_;

    if (!active_) {
      goal_x_ = nx;
      goal_y_ = ny;
    } else {
      goal_x_ = goal_alpha_ * nx + (1.0 - goal_alpha_) * goal_x_;
      goal_y_ = goal_alpha_ * ny + (1.0 - goal_alpha_) * goal_y_;
    }

    active_ = true;
    waiting_to_return_ = false;
    returning_home_ = false;

    if (!was_active || std::hypot(goal_x_ - last_logged_x_, goal_y_ - last_logged_y_) > 0.04) {
      last_logged_x_ = goal_x_;
      last_logged_y_ = goal_y_;
      RCLCPP_INFO(
        this->get_logger(),
        "Target -> (x=%.3f m, y=%.3f m)", goal_x_, goal_y_);
    }
  }

  void publish_zero() {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

  void control_loop() {
    if (!odom_received_) {
      return;
    }

    if (!active_) {
      if (auto_return_home_ && waiting_to_return_) {
        const double elapsed = (this->now() - reached_time_).seconds();
        if (elapsed >= return_delay_s_) {
          waiting_to_return_ = false;
          returning_home_ = true;
          goal_x_ = 0.0;
          goal_y_ = 0.0;
          active_ = true;
          RCLCPP_INFO(this->get_logger(), "Returning to Home (0.000, 0.000)");
        }
      }
      return;
    }

    const double ex = goal_x_ - x_;
    const double ey = goal_y_ - y_;
    const double dist = std::hypot(ex, ey);
    const double eyaw = wrap_angle(0.0 - yaw_);

    if (dist < pos_tol_ && std::abs(eyaw) < yaw_tol_) {
      publish_zero();
      active_ = false;
      RCLCPP_INFO(
        this->get_logger(),
        "%s reached! Pose=(x=%.3f, y=%.3f, yaw=%.1f deg)",
        returning_home_ ? "Home" : "Target",
        x_, y_, yaw_ * 180.0 / M_PI);

      if (auto_return_home_ && !returning_home_ && std::hypot(x_, y_) > 0.05) {
        waiting_to_return_ = true;
        reached_time_ = this->now();
      }
      returning_home_ = false;
      return;
    }

    // Rotate world-frame error into robot body frame (FLU)
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    const double ex_body =  ex * c + ey * s;
    const double ey_body = -ex * s + ey * c;

    double vx = kp_lin_ * ex_body;
    double vy = kp_lin_ * ey_body;

    // Preserve travel direction when clamping linear speed
    const double speed = std::hypot(vx, vy);
    if (speed > max_lin_vel_) {
      const double scale = max_lin_vel_ / speed;
      vx *= scale;
      vy *= scale;
    }

    const double wz = std::clamp(kp_yaw_ * eyaw, -max_yaw_vel_, max_yaw_vel_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  double kp_lin_;
  double kp_yaw_;
  double max_lin_vel_;
  double max_yaw_vel_;
  double pos_tol_;
  double yaw_tol_;
  double goal_alpha_;
  bool auto_return_home_;
  double return_delay_s_;

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  bool odom_received_{false};

  double goal_x_{0.0};
  double goal_y_{0.0};
  double last_logged_x_{0.0};
  double last_logged_y_{0.0};
  bool active_{false};
  bool waiting_to_return_{false};
  bool returning_home_{false};
  rclcpp::Time reached_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GotoGoalNode>());
  rclcpp::shutdown();
  return 0;
}