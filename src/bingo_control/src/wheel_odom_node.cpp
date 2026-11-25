#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

class WheelOdomNode : public rclcpp::Node {
public:
  WheelOdomNode() : Node("wheel_odom_node") {
    ticks_per_rev_ = this->declare_parameter<double>("ticks_per_rev", 979.0);
    wheel_radius_  = this->declare_parameter<double>("wheel_radius", 0.0330);
    lx_            = this->declare_parameter<double>("lx", 0.08875);
    ly_            = this->declare_parameter<double>("ly", 0.1270);
    odom_frame_    = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_    = this->declare_parameter<std::string>("base_frame", "base_link");

    rad_per_tick_ = (2.0 * M_PI) / ticks_per_rev_;

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
    ticks_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/wheel_ticks", 10,
      std::bind(&WheelOdomNode::ticks_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "wheel_odom_node started (TPR=%.1f, r=%.4fm, lx=%.5fm, ly=%.4fm)",
      ticks_per_rev_, wheel_radius_, lx_, ly_);
  }

private:
  void ticks_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
      return;
    }

    const rclcpp::Time now = this->now();
    std::array<int32_t, 4> curr = {
      msg->data[0], msg->data[1], msg->data[2], msg->data[3]
    };

    if (!initialized_) {
      prev_ticks_ = curr;
      last_time_ = now;
      initialized_ = true;
      return;
    }

    const double dt = (now - last_time_).seconds();
    if (dt <= 1e-4) {
      return;
    }

    std::array<int32_t, 4> delta{};
    bool jump_detected = false;
    for (size_t i = 0; i < 4; ++i) {
      delta[i] = curr[i] - prev_ticks_[i];
      if (std::abs(delta[i]) > 5000) {
        jump_detected = true;
      }
    }

    prev_ticks_ = curr;
    last_time_ = now;

    if (jump_detected || dt > 0.5) {
      RCLCPP_WARN(
        this->get_logger(),
        "Tick jump or timeout (dt=%.3fs) detected; resyncing wheel counters.", dt);
      return;
    }

    // Wheel angular displacements [rad]: 0=FL, 1=FR, 2=RR, 3=RL
    const double d0 = static_cast<double>(delta[0]) * rad_per_tick_;
    const double d1 = static_cast<double>(delta[1]) * rad_per_tick_;
    const double d2 = static_cast<double>(delta[2]) * rad_per_tick_;
    const double d3 = static_cast<double>(delta[3]) * rad_per_tick_;

    // Forward mecanum kinematics (body frame FLU)
    const double dx_body = (wheel_radius_ / 4.0) * (d0 + d1 + d2 + d3);
    const double dy_body = (wheel_radius_ / 4.0) * (-d0 + d1 - d2 + d3);
    const double dtheta  = (wheel_radius_ / (4.0 * (lx_ + ly_))) * (-d0 + d1 + d2 - d3);

    // Midpoint (RK2) integration into odom frame
    const double yaw_mid = yaw_ + 0.5 * dtheta;
    x_ += dx_body * std::cos(yaw_mid) - dy_body * std::sin(yaw_mid);
    y_ += dx_body * std::sin(yaw_mid) + dy_body * std::cos(yaw_mid);
    yaw_ = std::atan2(std::sin(yaw_ + dtheta), std::cos(yaw_ + dtheta));

    // Populate nav_msgs/Odometry (no TF broadcast; EKF will own odom -> base_link)
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw_);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw_);

    odom.twist.twist.linear.x  = dx_body / dt;
    odom.twist.twist.linear.y  = dy_body / dt;
    odom.twist.twist.linear.z  = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = dtheta / dt;

    // EKF-ready diagonal covariances (higher lateral/yaw variance for mecanum slip)
    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0]  = 0.01;   // x
    odom.pose.covariance[7]  = 0.02;   // y
    odom.pose.covariance[14] = 1e6;    // z (unused in 2D)
    odom.pose.covariance[21] = 1e6;    // roll
    odom.pose.covariance[28] = 1e6;    // pitch
    odom.pose.covariance[35] = 0.05;   // yaw

    odom.twist.covariance.fill(0.0);
    odom.twist.covariance[0]  = 0.01;  // vx
    odom.twist.covariance[7]  = 0.04;  // vy (mecanum roller slip)
    odom.twist.covariance[14] = 1e6;   // vz
    odom.twist.covariance[21] = 1e6;   // wx
    odom.twist.covariance[28] = 1e6;   // wy
    odom.twist.covariance[35] = 0.05;  // wz

    odom_pub_->publish(odom);
  }

  double ticks_per_rev_;
  double wheel_radius_;
  double lx_;
  double ly_;
  double rad_per_tick_;
  std::string odom_frame_;
  std::string base_frame_;

  bool initialized_ = false;
  std::array<int32_t, 4> prev_ticks_{0, 0, 0, 0};
  rclcpp::Time last_time_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ticks_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomNode>());
  rclcpp::shutdown();
  return 0;
}