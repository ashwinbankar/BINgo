#ifndef BINGO_BRINGUP__BINGO_CONTROL_PANEL_HPP_
#define BINGO_BRINGUP__BINGO_CONTROL_PANEL_HPP_

#include <array>
#include <memory>
#include <mutex>

#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace bingo_bringup
{

class BingoControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BingoControlPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onStartLidar();
  void onStopLidar();
  void onResetOdom();
  void onStopAllMotors();
  void onPublishTimer();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  QLabel * status_label_{nullptr};
  QLabel * pose_label_{nullptr};
  QLabel * vel_label_{nullptr};
  std::array<QSlider *, 4> sliders_{};
  std::array<QLabel *, 4> value_labels_{};
  QTimer * publish_timer_{nullptr};

  std::mutex odom_mutex_;
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_deg_{0.0};
  double odom_vx_{0.0};
  double odom_vy_{0.0};
  double odom_wz_{0.0};
  bool odom_received_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_lidar_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_lidar_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_odom_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
};

}  // namespace bingo_bringup

#endif  // BINGO_BRINGUP__BINGO_CONTROL_PANEL_HPP_