#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <sensor_msgs/msg/imu.h>

// IMU
Adafruit_MPU6050 mpu;
bool mpu_ok = false;

// Teensy 4.1 Pin Mapping (Schematic Rev. 2.0)
// Mapped to: M0 = FL, M1 = FR, M2 = RR, M3 = RL (matches Schematic Rev. 2.0)
const uint8_t DIR_PINS[4] = {10, 8, 12, 25};
const uint8_t PWM_PINS[4] = {11, 9, 24, 28};

// Baseline polarity for dry-run check
const int8_t MOTOR_POLARITY[4] = {1, -1, -1, 1};
const uint8_t MAX_PWM = 255;                   // Keep 160 ~63% max duty for safe bench test and 255 for ~100% duty
const unsigned long CMD_TIMEOUT_MS = 500;      // Watchdog timeout

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_timer_t imu_timer;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_subscription_t motor_subscriber;
std_msgs__msg__Float32MultiArray motor_msg;
float motor_msg_buffer[4] = {0.0f, 0.0f, 0.0f, 0.0f};
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
AgentState state = WAITING_AGENT;
unsigned long last_ping_ms = 0;
unsigned long last_cmd_ms = 0;

void set_motor(uint8_t idx, float speed_norm) {
  speed_norm = constrain(speed_norm, -1.0f, 1.0f) * MOTOR_POLARITY[idx];
  if (fabsf(speed_norm) < 0.02f) {
    analogWrite(PWM_PINS[idx], 0);
    digitalWrite(DIR_PINS[idx], LOW);
    return;
  }
  digitalWrite(DIR_PINS[idx], speed_norm >= 0.0f ? HIGH : LOW);
  uint8_t duty = (uint8_t)(fabsf(speed_norm) * MAX_PWM);
  analogWrite(PWM_PINS[idx], duty);
}

void stop_all_motors() {
  for (uint8_t i = 0; i < 4; i++) {
    set_motor(i, 0.0f);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis();

  float vx = twist->linear.x;
  float vy = twist->linear.y;
  float wz = twist->angular.z;

// M0: Front-Left (FL), M1: Front-Right (FR), M2: Rear-Right (RR), M3: Rear-Left (RL)
float w[4];
w[0] = vx - vy - wz;  // M0: FL
w[1] = vx + vy + wz;  // M1: FR
w[2] = vx - vy + wz;  // M2: RR
w[3] = vx + vy - wz;  // M3: RL

  float max_mag = 1.0f;
  for (uint8_t i = 0; i < 4; i++) {
    if (fabsf(w[i]) > max_mag) max_mag = fabsf(w[i]);
  }
  for (uint8_t i = 0; i < 4; i++) {
    set_motor(i, w[i] / max_mag);
  }

  digitalWrite(LED_BUILTIN, (fabsf(vx) > 0.01f || fabsf(vy) > 0.01f || fabsf(wz) > 0.01f) ? HIGH : LOW);
}

void motor_cmd_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * arr = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (arr->data.size < 4) return;

  last_cmd_ms = millis();
  bool any_active = false;
  for (uint8_t i = 0; i < 4; i++) {
    set_motor(i, arr->data.data[i]);
    if (fabsf(arr->data.data[i]) > 0.02f) any_active = true;
  }
  digitalWrite(LED_BUILTIN, any_active ? HIGH : LOW);
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL || !mpu_ok) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Frame ID
  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = 8;
  imu_msg.header.frame_id.capacity = 9;

  // Orientation not computed on Teensy (-1 tells ROS to ignore it)
  imu_msg.orientation_covariance[0] = -1.0;

  // Angular velocity (rad/s) and linear acceleration (m/s^2)
  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "teensy_motor_node", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) return false;

  // Bind pre-allocated buffer so micro-ROS can deserialize incoming float arrays
  motor_msg.data.data = motor_msg_buffer;
  motor_msg.data.size = 0;
  motor_msg.data.capacity = 4;

  if (rclc_subscription_init_default(
        &motor_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/motor_cmd") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
      &imu_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/data_raw") != RCL_RET_OK) return false;

  if (rclc_timer_init_default(
      &imu_timer, &support, RCL_MS_TO_NS(20), imu_timer_callback) != RCL_RET_OK) return false;

  // Increased handle count from 1 to 2 for both subscriptions
  if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &imu_timer) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  last_cmd_ms = millis();
  return true;
}

void destroy_entities() {
  stop_all_motors();
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&subscriber, &node);
  rcl_subscription_fini(&motor_subscriber, &node);
  rcl_timer_fini(&imu_timer);
  rcl_publisher_fini(&imu_publisher, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(PWM_PINS[i], OUTPUT);
    analogWriteFrequency(PWM_PINS[i], 20000);
  }
  stop_all_motors();

  Wire.begin();
  mpu_ok = mpu.begin(0x68, &Wire);
  if (mpu_ok) {
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (millis() - last_ping_ms > 500) {
        last_ping_ms = millis();
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          state = AGENT_AVAILABLE;
        }
      }
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      break;

    case AGENT_CONNECTED:
      if (millis() - last_ping_ms > 1000) {
        last_ping_ms = millis();
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          state = AGENT_DISCONNECTED;
          break;
        }
      }
      if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
        stop_all_motors();
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
