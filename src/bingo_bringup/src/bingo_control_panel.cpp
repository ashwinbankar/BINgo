#include "bingo_bringup/bingo_control_panel.hpp"

#include <cmath>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace bingo_bringup
{

BingoControlPanel::BingoControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * main_layout = new QVBoxLayout;

  // --- LiDAR Controls ---
  main_layout->addWidget(new QLabel("<b>RPLIDAR A1 Motor</b>"));
  auto * lidar_row = new QHBoxLayout;
  auto * start_btn = new QPushButton("Start LiDAR");
  auto * stop_btn = new QPushButton("Stop LiDAR");
  lidar_row->addWidget(start_btn);
  lidar_row->addWidget(stop_btn);
  main_layout->addLayout(lidar_row);

  status_label_ = new QLabel("Status: Ready");
  main_layout->addWidget(status_label_);

  // --- Odometry Telemetry & Reset ---
  main_layout->addSpacing(8);
  main_layout->addWidget(new QLabel("<b>Odometry Telemetry (/wheel_odom)</b>"));
  pose_label_ = new QLabel("Pose: x=0.00 m  y=0.00 m  yaw=0.0°");
  vel_label_  = new QLabel("Vel:  vx=0.00  vy=0.00 m/s  wz=0.00 rad/s");
  auto * reset_odom_btn = new QPushButton("Reset Odometry");
  main_layout->addWidget(pose_label_);
  main_layout->addWidget(vel_label_);
  main_layout->addWidget(reset_odom_btn);

  // --- Individual Motor Sliders ---
  main_layout->addSpacing(8);
  main_layout->addWidget(new QLabel("<b>Motor Speed & Direction (-1.0 to 1.0)</b>"));

  const char * names[4] = {"M0 (FL)", "M1 (FR)", "M2 (RR)", "M3 (RL)"};
  for (int i = 0; i < 4; ++i) {
    auto * row = new QHBoxLayout;
    auto * name_lbl = new QLabel(names[i]);
    name_lbl->setFixedWidth(55);

    sliders_[i] = new QSlider(Qt::Horizontal);
    sliders_[i]->setRange(-100, 100);
    sliders_[i]->setValue(0);

    value_labels_[i] = new QLabel("0.00");
    value_labels_[i]->setFixedWidth(38);

    connect(sliders_[i], &QSlider::valueChanged, this, [this, i](int val) {
      value_labels_[i]->setText(QString::number(val / 100.0, 'f', 2));
    });

    row->addWidget(name_lbl);
    row->addWidget(sliders_[i]);
    row->addWidget(value_labels_[i]);
    main_layout->addLayout(row);
  }

  // --- Emergency Stop Button ---
  auto * stop_motors_btn = new QPushButton("STOP ALL MOTORS");
  stop_motors_btn->setStyleSheet("background-color: #a83232; color: white; font-weight: bold; padding: 6px;");
  main_layout->addWidget(stop_motors_btn);

  setLayout(main_layout);

  connect(start_btn, &QPushButton::clicked, this, &BingoControlPanel::onStartLidar);
  connect(stop_btn, &QPushButton::clicked, this, &BingoControlPanel::onStopLidar);
  connect(reset_odom_btn, &QPushButton::clicked, this, &BingoControlPanel::onResetOdom);
  connect(stop_motors_btn, &QPushButton::clicked, this, &BingoControlPanel::onStopAllMotors);

  // 10 Hz timer: feeds Teensy 500 ms watchdog for active sliders & refreshes telemetry UI
  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &BingoControlPanel::onPublishTimer);
  publish_timer_->start(100);
}

void BingoControlPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  motor_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_cmd", 10);
  estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/estop", 10);
  odom_sub_  = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/wheel_odom", 10,
    std::bind(&BingoControlPanel::odomCallback, this, std::placeholders::_1));
  start_lidar_client_ = node_->create_client<std_srvs::srv::Empty>("/start_motor");
  stop_lidar_client_  = node_->create_client<std_srvs::srv::Empty>("/stop_motor");
  reset_odom_client_  = node_->create_client<std_srvs::srv::Trigger>("/reset_odom");
}

void BingoControlPanel::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  const double yaw_rad = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  odom_yaw_deg_ = yaw_rad * (180.0 / M_PI);
  odom_vx_ = msg->twist.twist.linear.x;
  odom_vy_ = msg->twist.twist.linear.y;
  odom_wz_ = msg->twist.twist.angular.z;
  odom_received_ = true;
}

void BingoControlPanel::onStartLidar()
{
  if (!start_lidar_client_->service_is_ready()) {
    status_label_->setText("Status: /start_motor unavailable");
    return;
  }
  start_lidar_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  status_label_->setText("Status: LiDAR Started");
}

void BingoControlPanel::onStopLidar()
{
  if (!stop_lidar_client_->service_is_ready()) {
    status_label_->setText("Status: /stop_motor unavailable");
    return;
  }
  stop_lidar_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  status_label_->setText("Status: LiDAR Stopped");
}

void BingoControlPanel::onResetOdom()
{
  if (!reset_odom_client_->service_is_ready()) {
    status_label_->setText("Status: /reset_odom unavailable");
    return;
  }
  reset_odom_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_yaw_deg_ = 0.0;
    odom_vx_ = 0.0;
    odom_vy_ = 0.0;
    odom_wz_ = 0.0;
  }
  pose_label_->setText("Pose: x=0.00 m  y=0.00 m  yaw=0.0°");
  vel_label_->setText("Vel:  vx=0.00  vy=0.00 m/s  wz=0.00 rad/s");
  status_label_->setText("Status: Odometry Reset");
}

void BingoControlPanel::onStopAllMotors()
{
  for (int i = 0; i < 4; ++i) {
    sliders_[i]->setValue(0);
  }
  if (estop_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    estop_pub_->publish(msg);
  }
  status_label_->setText("Status: E-STOP Triggered");
}

void BingoControlPanel::onPublishTimer()
{
  // Refresh telemetry labels on the Qt GUI thread at 10 Hz
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_received_) {
      pose_label_->setText(
        QString("Pose: x=%1 m  y=%2 m  yaw=%3°")
          .arg(odom_x_, 0, 'f', 2)
          .arg(odom_y_, 0, 'f', 2)
          .arg(odom_yaw_deg_, 0, 'f', 1));
      vel_label_->setText(
        QString("Vel:  vx=%1  vy=%2 m/s  wz=%3 rad/s")
          .arg(odom_vx_, 0, 'f', 2)
          .arg(odom_vy_, 0, 'f', 2)
          .arg(odom_wz_, 0, 'f', 2));
    }
  }

  if (!motor_pub_) return;

  bool any_active = false;
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(4);

  for (int i = 0; i < 4; ++i) {
    msg.data[i] = static_cast<float>(sliders_[i]->value()) / 100.0f;
    if (sliders_[i]->value() != 0) any_active = true;
  }

  if (any_active) {
    motor_pub_->publish(msg);
  }
}

}  // namespace bingo_bringup

PLUGINLIB_EXPORT_CLASS(bingo_bringup::BingoControlPanel, rviz_common::Panel)