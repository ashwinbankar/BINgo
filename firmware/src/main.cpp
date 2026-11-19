#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
State state = WAITING_AGENT;
uint32_t last_check = 0;

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * incoming = (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(LED_BUILTIN, (incoming->linear.x > 0.0) ? HIGH : LOW);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "teensy_dummy_node", "", &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg,
        &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (millis() - last_check > 500) {
        last_check = millis();
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (millis() - last_check > 500) {
        last_check = millis();
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) state = AGENT_DISCONNECTED;
      }
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;

    case AGENT_DISCONNECTED:
      digitalWrite(LED_BUILTIN, LOW);
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
