#ifndef BINGO_BRINGUP__BINGO_CONTROL_PANEL_HPP_
#define BINGO_BRINGUP__BINGO_CONTROL_PANEL_HPP_

#include <array>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>

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
  void onStopAllMotors();
  void onPublishTimer();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_lidar_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_lidar_client_;

  std::array<QSlider *, 4> sliders_;
  std::array<QLabel *, 4> value_labels_;
  QLabel * status_label_;
  QTimer * publish_timer_;
};

}  // namespace bingo_bringup

#endif  // BINGO_BRINGUP__BINGO_CONTROL_PANEL_HPP_