#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class WheelOdomNode : public rclcpp::Node {
public:
  WheelOdomNode() : Node("wheel_odom_node") {
    ticks_per_rev_ = this->declare_parameter<double>("ticks_per_rev", 979.0);
    wheel_radius_  = this->declare_parameter<double>("wheel_radius", 0.0330);
    lx_            = this->declare_parameter<double>("lx", 0.08875);
    ly_            = this->declare_parameter<double>("ly", 0.1270);
    odom_frame_    = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_    = this->declare_parameter<std::string>("base_frame", "base_link");
    // Default false: the EKF (robot_localization) owns odom -> base_link
    publish_tf_    = this->declare_parameter<bool>("publish_tf", false);

    rad_per_tick_ = (2.0 * M_PI) / ticks_per_rev_;

    odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    ticks_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/wheel_ticks", 10,
      std::bind(&WheelOdomNode::ticks_callback, this, std::placeholders::_1));

    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/reset_odom",
      std::bind(
        &WheelOdomNode::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Keep wheel joints alive at 10 Hz when Teensy is powered off
    idle_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WheelOdomNode::idle_timer_callback, this));

    last_pub_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "wheel_odom_node started (TPR=%.1f, r=%.4fm, lx=%.5fm, ly=%.4fm, publish_tf=%s)",
      ticks_per_rev_, wheel_radius_, lx_, ly_, publish_tf_ ? "true" : "false");
  }

private:
  void publish_tf_and_joints(const rclcpp::Time & stamp) {
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = 0.0;
      tf_msg.transform.rotation.y = 0.0;
      tf_msg.transform.rotation.z = std::sin(0.5 * yaw_);
      tf_msg.transform.rotation.w = std::cos(0.5 * yaw_);
      tf_broadcaster_->sendTransform(tf_msg);
    }

    // URDF wheel joint axes in SolidWorks-rotated Chassis_link: FL(-), FR(+), RR(+), RL(-)
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {
      "front_left_wheel_joint",
      "front_right_wheel_joint",
      "back_right_wheel_joint",
      "back_left_wheel_joint"
    };
    js.position = {
      -wheel_angles_[0],
       wheel_angles_[1],
       wheel_angles_[2],
      -wheel_angles_[3]
    };
    joint_pub_->publish(js);
  }

  void idle_timer_callback() {
    const rclcpp::Time now = this->now();
    if (!initialized_ || (now - last_pub_time_).seconds() > 0.25) {
      publish_tf_and_joints(now);
    }
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    wheel_angles_.fill(0.0);
    publish_tf_and_joints(this->now());
    res->success = true;
    res->message = "Odometry reset to (0, 0, 0)";
    RCLCPP_INFO(this->get_logger(), "Odometry reset to (0, 0, 0) via /reset_odom");
  }

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
      last_pub_time_ = now;
      initialized_ = true;
      publish_tf_and_joints(now);
      return;
    }

    const double dt = (now - last_time_).seconds();
    if (dt < 0.010) {
      return;  // burst arrival: keep prev_ticks_/last_time_ so the ticks accumulate into the next sample
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
    last_pub_time_ = now;

    if (jump_detected || dt > 0.5) {
      RCLCPP_WARN(
        this->get_logger(),
        "Tick jump or timeout (dt=%.3fs) detected; resyncing wheel counters.", dt);
      publish_tf_and_joints(now);
      return;
    }

    // Wheel angular displacements [rad]: 0=FL, 1=FR, 2=RR, 3=RL
    const double d0 = static_cast<double>(delta[0]) * rad_per_tick_;
    const double d1 = static_cast<double>(delta[1]) * rad_per_tick_;
    const double d2 = static_cast<double>(delta[2]) * rad_per_tick_;
    const double d3 = static_cast<double>(delta[3]) * rad_per_tick_;

    wheel_angles_[0] += d0;
    wheel_angles_[1] += d1;
    wheel_angles_[2] += d2;
    wheel_angles_[3] += d3;

    // Forward mecanum kinematics (body frame FLU)
    const double dx_body = (wheel_radius_ / 4.0) * (d0 + d1 + d2 + d3);
    const double dy_body = (wheel_radius_ / 4.0) * (-d0 + d1 - d2 + d3);
    const double dtheta  = (wheel_radius_ / (4.0 * (lx_ + ly_))) * (-d0 + d1 + d2 - d3);

    // Midpoint (RK2) integration into odom frame
    const double yaw_mid = yaw_ + 0.5 * dtheta;
    x_ += dx_body * std::cos(yaw_mid) - dy_body * std::sin(yaw_mid);
    y_ += dx_body * std::sin(yaw_mid) + dy_body * std::cos(yaw_mid);
    yaw_ = std::atan2(std::sin(yaw_ + dtheta), std::cos(yaw_ + dtheta));

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

    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.02;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 0.05;

    odom.twist.covariance.fill(0.0);
    odom.twist.covariance[0]  = 0.01;
    odom.twist.covariance[7]  = 0.04;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 0.05;

    odom_pub_->publish(odom);
    publish_tf_and_joints(now);
  }

  double ticks_per_rev_;
  double wheel_radius_;
  double lx_;
  double ly_;
  double rad_per_tick_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;

  bool initialized_ = false;
  std::array<int32_t, 4> prev_ticks_{0, 0, 0, 0};
  std::array<double, 4> wheel_angles_{0.0, 0.0, 0.0, 0.0};
  rclcpp::Time last_time_;
  rclcpp::Time last_pub_time_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ticks_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr idle_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomNode>());
  rclcpp::shutdown();
  return 0;
}