
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include "driver/pcnt.h"

// ==== Pin cấu hình ====
#define PUL_PINS   {18, 14, 32, 25}
#define DIR_PINS   {19, 27, 33, 26}
#define CHANNELS   {0, 1, 2, 3}
#define ENCODER_PINS {4, 5, 16, 17}

const int pwm_pins[4] = PUL_PINS;
const int dir_pins[4] = DIR_PINS;
const int encoder_pins[4] = ENCODER_PINS;
const int channels[4] = CHANNELS;
const pcnt_unit_t PCNT_UNITS[4] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};
volatile long prev_counts[4] = {0, 0, 0, 0};

// ==== Thông số robot ====
const float WHEEL_RADIUS = 0.05;
const float ROBOT_LENGTH = 0.2;
const float ROBOT_WIDTH  = 0.15;
const float PULSES_PER_REV = 5000.0;
const float GEAR_RATIO = 1.0;

// ==== micro-ROS ====
rcl_node_t node;
rcl_subscription_t freq_sub;
rcl_publisher_t log_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t freq_pub;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__String input_msg;
std_msgs__msg__String log_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Float32MultiArray freq_msg;
float freq_data[4];

float x = 0.0, y = 0.0, theta = 0.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) while(1){delay(1000);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void publish_log(const char* format, ...) {
  static char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  strncpy(log_msg.data.data, buffer, log_msg.data.capacity - 1);
  log_msg.data.data[log_msg.data.capacity - 1] = '\0';
  log_msg.data.size = strlen(log_msg.data.data);

  rcl_publish(&log_pub, &log_msg, NULL);
}

// ==== PCNT Encoder Setup ====
void setup_pcnt_encoder(int i) {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = encoder_pins[i],
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .channel = PCNT_CHANNEL_0,
    .unit = PCNT_UNITS[i],
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768
  };
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNITS[i]);
  pcnt_counter_clear(PCNT_UNITS[i]);
  pcnt_counter_resume(PCNT_UNITS[i]);
}

// ==== Motor PWM control ====
void set_motor_freq_and_dir(long freqs[4], int dirs[4]) {
  for (int i = 0; i < 4; i++) {
    int res;
    if (freqs[i] >= 500000) res = 6;
    else if (freqs[i] >= 250000) res = 7;
    else if (freqs[i] >= 125000) res = 8;
    else if (freqs[i] >= 60000)  res = 9;
    else res = 10;

    int max_duty = (1 << res) - 1;
    ledcSetup(channels[i], freqs[i], res);
    ledcAttachPin(pwm_pins[i], channels[i]);
    ledcWrite(channels[i], max_duty / 2);  // 50% duty

    digitalWrite(dir_pins[i], dirs[i]);
    publish_log("[LOG] Motor %d: Freq=%ld Hz, Dir=%d", i + 1, freqs[i], dirs[i]);
  }
}

// ==== Callback từ freq_cmd ====
void freq_cmd_callback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  publish_log("[INFO] Received command: %s", msg->data.data);

  long freqs[4];
  int dirs[4];

  if (sscanf(msg->data.data, "SET %ld %d %ld %d %ld %d %ld %d",
             &freqs[0], &dirs[0], &freqs[1], &dirs[1],
             &freqs[2], &dirs[2], &freqs[3], &dirs[3]) == 8) {
    set_motor_freq_and_dir(freqs, dirs);
  } else {
    publish_log("[ERROR] Invalid format: %s", msg->data.data);
  }
}

// ==== Odometry tính từ encoder ====
void compute_and_publish_odom(rcl_timer_t * timer, int64_t last_call_time) {
  const float dt = 0.1f;
  long delta[4];
  float freq[4];

  for (int i = 0; i < 4; i++) {
    int16_t count;
    pcnt_get_counter_value(PCNT_UNITS[i], &count);
    pcnt_counter_clear(PCNT_UNITS[i]);
    delta[i] = count - prev_counts[i];
    prev_counts[i] = count;
    freq[i] = delta[i] / dt;
    freq_data[i] = freq[i];
  }

  float v1 = (2 * PI * WHEEL_RADIUS * freq[0]) / (PULSES_PER_REV * GEAR_RATIO);
  float v2 = (2 * PI * WHEEL_RADIUS * freq[1]) / (PULSES_PER_REV * GEAR_RATIO);
  float v3 = (2 * PI * WHEEL_RADIUS * freq[2]) / (PULSES_PER_REV * GEAR_RATIO);
  float v4 = (2 * PI * WHEEL_RADIUS * freq[3]) / (PULSES_PER_REV * GEAR_RATIO);

  float vx = (v1 + v2 + v3 + v4) / 4.0;
  float vy = (-v1 + v2 + v3 - v4) / 4.0;
  float omega = (-v1 + v2 - v3 + v4) / (4.0 * (ROBOT_LENGTH + ROBOT_WIDTH));

  float dx = vx * dt * cos(theta) - vy * dt * sin(theta);
  float dy = vx * dt * sin(theta) + vy * dt * cos(theta);
  float dtheta = omega * dt;

  x += dx;
  y += dy;
  theta += dtheta;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;

  rcl_publish(&odom_pub, &odom_msg, NULL);
  freq_msg.data.data = freq_data;
  freq_msg.data.size = 4;
  rcl_publish(&freq_pub, &freq_msg, NULL);
}

// ==== Setup chính ====
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(1000);

  for (int i = 0; i < 4; i++) {
    pinMode(dir_pins[i], OUTPUT);
    digitalWrite(dir_pins[i], LOW);
    pinMode(encoder_pins[i], INPUT);
    setup_pcnt_encoder(i);
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &freq_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "freq_cmd"));

  RCCHECK(rclc_publisher_init_default(
    &log_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "output_log"));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  RCCHECK(rclc_publisher_init_default(
    &freq_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_freq"));

  input_msg.data.data = (char*)malloc(128);
  input_msg.data.size = 0;
  input_msg.data.capacity = 128;

  log_msg.data.data = (char*)malloc(128);
  log_msg.data.size = 0;
  log_msg.data.capacity = 128;

  freq_msg.data.data = freq_data;
  freq_msg.data.size = 4;
  freq_msg.data.capacity = 4;

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    compute_and_publish_odom
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &freq_sub, &input_msg, &freq_cmd_callback, ON_NEW_DATA));

  publish_log("[INFO] Node ready.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
