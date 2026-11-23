#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// Teensy 4.1 Pin Mapping (Schematic Rev. 2.0)
const uint8_t DIR_PINS[4] = {8, 10, 12, 25};   // M0_DIR, M1_DIR, M2_DIR, M3_DIR
const uint8_t PWM_PINS[4] = {9, 11, 24, 28};   // M0_PWM, M1_PWM, M2_PWM, M3_PWM

// Flip to -1 for any motor that spins backward on a forward command
const int8_t MOTOR_POLARITY[4] = {1, 1, 1, 1};

const uint8_t MAX_PWM = 160;                   // ~63% max duty for safe bench test
const unsigned long CMD_TIMEOUT_MS = 500;      // Watchdog timeout

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

  // Left/Right driver grouping:
  // M0: Front-Left (D0), M1: Rear-Left (D0), M2: Front-Right (D1), M3: Rear-Right (D1)
  float w[4];
  w[0] = vx - vy - wz;
  w[1] = vx + vy - wz;
  w[2] = vx + vy + wz;
  w[3] = vx - vy + wz;

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

  // Increased handle count from 1 to 2 for both subscriptions
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
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
