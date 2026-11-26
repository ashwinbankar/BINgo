#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Encoder.h>

// Quadrature Encoders (Direct 3.3V on pins 0-7)
Encoder enc[4] = {
  Encoder(0, 1),  // E0: FL
  Encoder(2, 3),  // E1: FR
  Encoder(4, 5),  // E2: RR
  Encoder(6, 7)   // E3: RL
};
const int8_t ENC_POLARITY[4] = {-1, 1, 1, -1};

// Teensy 4.1 Pin Mapping (Schematic Rev. 2.0)
// M0 = FL, M1 = FR, M2 = RR, M3 = RL
const uint8_t DIR_PINS[4] = {10, 8, 12, 25};
const uint8_t PWM_PINS[4] = {11, 9, 24, 28};
const int8_t MOTOR_POLARITY[4] = {1, -1, -1, 1};
const uint8_t MAX_PWM = 255;
const unsigned long CMD_TIMEOUT_MS = 500;

// Locked Kinematics & Encoder Constants
const float TICKS_PER_REV   = 979.0f;
const float RAD_PER_TICK    = 6.28318530718f / TICKS_PER_REV;
const float WHEEL_RADIUS    = 0.0330f;          // m (66 mm OD)
const float LX              = 0.08875f;         // m (half-wheelbase)
const float LY              = 0.1270f;          // m (half-track)
const float L_SUM           = LX + LY;          // 0.21575 m
const float MAX_WHEEL_RAD_S = 28.0f;            // Closed-loop cap (~0.924 m/s)

// Stage 1: Universal Trapezoidal Velocity Profiler (rad/s)
const float CONTROL_DT        = 0.02f;          // 50 Hz nominal
const float WHEEL_ACCEL_LIMIT = 56.0f;          // rad/s^2 (0 -> 28 in 0.50 s)
const float WHEEL_DECEL_LIMIT = 84.0f;          // rad/s^2 (28 -> 0 in 0.33 s)
float target_rad_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float ramped_rad_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Stage 2: Per-Wheel Velocity Feedforward + PID (Starter Bench Gains)
const float K_FF    = 0.0315f;                  // 1.0 / 31.76 rad/s (measured no-load)
const float K_P     = 0.015f;                   // norm / (rad/s)
const float K_I     = 0.080f;                   // norm / rad
const float K_D     = 0.000f;                   // norm * s / rad
const float I_CLAMP = 0.40f;                    // Anti-windup clamp (max +-40% PWM)

int32_t prev_ticks[4]      = {0, 0, 0, 0};
float pid_integral[4]      = {0.0f, 0.0f, 0.0f, 0.0f};
float pid_prev_error[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned long last_ctrl_us = 0;

rcl_publisher_t ticks_publisher;
std_msgs__msg__Int32MultiArray ticks_msg;
int32_t ticks_buffer[4] = {0, 0, 0, 0};
rcl_timer_t control_timer;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_subscription_t motor_subscriber;
std_msgs__msg__Float32MultiArray motor_msg;
float motor_msg_buffer[4] = {0.0f, 0.0f, 0.0f, 0.0f};
rcl_subscription_t estop_subscriber;
std_msgs__msg__Bool estop_msg;

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

void hard_stop_all_motors() {
  for (uint8_t i = 0; i < 4; i++) {
    target_rad_s[i] = 0.0f;
    ramped_rad_s[i] = 0.0f;
    pid_integral[i] = 0.0f;
    pid_prev_error[i] = 0.0f;
  }
  stop_all_motors();
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis();

  float vx = twist->linear.x;   // m/s
  float vy = twist->linear.y;   // m/s
  float wz = twist->angular.z;  // rad/s

  // Physical inverse mecanum kinematics (rad/s): M0=FL, M1=FR, M2=RR, M3=RL
  float w[4];
  w[0] = (vx - vy - L_SUM * wz) / WHEEL_RADIUS;
  w[1] = (vx + vy + L_SUM * wz) / WHEEL_RADIUS;
  w[2] = (vx - vy + L_SUM * wz) / WHEEL_RADIUS;
  w[3] = (vx + vy - L_SUM * wz) / WHEEL_RADIUS;

  // Proportional desaturation to preserve motion direction
  float max_mag = MAX_WHEEL_RAD_S;
  for (uint8_t i = 0; i < 4; i++) {
    if (fabsf(w[i]) > max_mag) max_mag = fabsf(w[i]);
  }
  float scale = MAX_WHEEL_RAD_S / max_mag;
  for (uint8_t i = 0; i < 4; i++) {
    target_rad_s[i] = w[i] * scale;
  }
}

void motor_cmd_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * arr = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (arr->data.size < 4) return;

  last_cmd_ms = millis();
  for (uint8_t i = 0; i < 4; i++) {
    target_rad_s[i] = constrain(arr->data.data[i], -1.0f, 1.0f) * MAX_WHEEL_RAD_S;
  }
}

void estop_callback(const void * msgin) {
  const std_msgs__msg__Bool * estop = (const std_msgs__msg__Bool *)msgin;
  if (estop->data) {
    hard_stop_all_motors();
  }
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  // Measure actual control dt
  unsigned long now_us = micros();
  float dt = (now_us - last_ctrl_us) * 1e-6f;
  last_ctrl_us = now_us;
  if (dt < 0.005f || dt > 0.1f) {
    dt = CONTROL_DT;
  }

  // 1) Read & publish calibrated cumulative quadrature ticks at 50 Hz
  int32_t curr_ticks[4];
  for (uint8_t i = 0; i < 4; i++) {
    curr_ticks[i] = (int32_t)(enc[i].read() * ENC_POLARITY[i]);
    ticks_buffer[i] = curr_ticks[i];
  }
  rcl_publish(&ticks_publisher, &ticks_msg, NULL);

  // 2) Stage 1 (Trapezoidal Profiler) + Stage 2 (Closed-Loop Per-Wheel Velocity PID)
  bool any_active = false;
  for (uint8_t i = 0; i < 4; i++) {
    int32_t delta_ticks = curr_ticks[i] - prev_ticks[i];
    prev_ticks[i] = curr_ticks[i];
    float meas_rad_s = (delta_ticks * RAD_PER_TICK) / dt;

    // Slew-rate limit target_rad_s -> ramped_rad_s
    float diff = target_rad_s[i] - ramped_rad_s[i];
    bool accelerating = (target_rad_s[i] * ramped_rad_s[i] >= 0.0f) &&
                        (fabsf(target_rad_s[i]) > fabsf(ramped_rad_s[i]));
    float max_step = (accelerating ? WHEEL_ACCEL_LIMIT : WHEEL_DECEL_LIMIT) * dt;

    if (fabsf(diff) <= max_step) {
      ramped_rad_s[i] = target_rad_s[i];
    } else {
      ramped_rad_s[i] += (diff > 0.0f ? max_step : -max_step);
    }

    // Clean zero-stop: kill PWM and clear integrator when stopped
    if (fabsf(target_rad_s[i]) < 0.05f && fabsf(ramped_rad_s[i]) < 0.05f) {
      pid_integral[i] = 0.0f;
      pid_prev_error[i] = 0.0f;
      set_motor(i, 0.0f);
      continue;
    }

    // Feedforward + PID
    float error = ramped_rad_s[i] - meas_rad_s;
    pid_integral[i] = constrain(pid_integral[i] + (K_I * error * dt), -I_CLAMP, I_CLAMP);
    float deriv = (error - pid_prev_error[i]) / dt;
    pid_prev_error[i] = error;

    float ff  = K_FF * ramped_rad_s[i];
    float out = constrain(ff + (K_P * error) + pid_integral[i] + (K_D * deriv), -1.0f, 1.0f);

    set_motor(i, out);
    any_active = true;
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

  motor_msg.data.data = motor_msg_buffer;
  motor_msg.data.size = 0;
  motor_msg.data.capacity = 4;

  if (rclc_subscription_init_default(
        &motor_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/motor_cmd") != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
        &estop_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/estop") != RCL_RET_OK) return false;

  ticks_msg.data.data = ticks_buffer;
  ticks_msg.data.size = 4;
  ticks_msg.data.capacity = 4;

  if (rclc_publisher_init_default(
      &ticks_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/wheel_ticks") != RCL_RET_OK) return false;

  if (rclc_timer_init_default(
      &control_timer, &support, RCL_MS_TO_NS(20), control_timer_callback) != RCL_RET_OK) return false;

  // 4 handles: 3 subscriptions (/cmd_vel, /motor_cmd, /estop) + 1 50 Hz control/ticks timer
  if (rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &estop_subscriber, &estop_msg, &estop_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  last_cmd_ms = millis();
  rmw_uros_sync_session(1000);

  hard_stop_all_motors();
  for (uint8_t i = 0; i < 4; i++) {
    enc[i].write(0);
    prev_ticks[i] = 0;
  }
  last_ctrl_us = micros();

  return true;
}

void destroy_entities() {
  hard_stop_all_motors();
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&subscriber, &node);
  rcl_subscription_fini(&motor_subscriber, &node);
  rcl_subscription_fini(&estop_subscriber, &node);
  rcl_timer_fini(&control_timer);
  rcl_publisher_fini(&ticks_publisher, &node);
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
  hard_stop_all_motors();

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
      // Smooth watchdog deceleration (profiler ramps ramped_rad_s -> 0)
      if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
        for (uint8_t i = 0; i < 4; i++) {
          target_rad_s[i] = 0.0f;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}