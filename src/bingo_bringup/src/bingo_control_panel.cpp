#include "bingo_bringup/bingo_control_panel.hpp"

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

  // --- Individual Motor Sliders ---
  main_layout->addSpacing(8);
  main_layout->addWidget(new QLabel("<b>Motor Speed & Direction (-1.0 to 1.0)</b>"));

  const char * names[4] = {"M0 (FL)", "M1 (RL)", "M2 (FR)", "M3 (RR)"};
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
  connect(stop_motors_btn, &QPushButton::clicked, this, &BingoControlPanel::onStopAllMotors);

  // Publish at 10 Hz whenever any slider is non-zero to feed the Teensy 500 ms watchdog
  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &BingoControlPanel::onPublishTimer);
  publish_timer_->start(100);
}

void BingoControlPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  motor_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_cmd", 10);
  start_lidar_client_ = node_->create_client<std_srvs::srv::Empty>("/start_motor");
  stop_lidar_client_ = node_->create_client<std_srvs::srv::Empty>("/stop_motor");
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

void BingoControlPanel::onStopAllMotors()
{
  for (int i = 0; i < 4; ++i) {
    sliders_[i]->setValue(0);
  }
  if (motor_pub_) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {0.0f, 0.0f, 0.0f, 0.0f};
    motor_pub_->publish(msg);
  }
}

void BingoControlPanel::onPublishTimer()
{
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