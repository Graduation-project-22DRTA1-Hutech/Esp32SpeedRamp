
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

// ==== Encoder PCNT ====
#define ENCODER_PINS {4, 5, 16, 17}
const int encoder_pins[4] = ENCODER_PINS;
const pcnt_unit_t PCNT_UNITS[4] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};
volatile long prev_counts[4] = {0, 0, 0, 0};

// ==== Odometry Parameters ====
const float WHEEL_RADIUS = 0.05;     // meters
const float ROBOT_LENGTH = 0.2;      // meters
const float ROBOT_WIDTH  = 0.15;     // meters
const float PULSES_PER_REV = 5000.0;
const float GEAR_RATIO = 1.0;

// ==== micro-ROS ====
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t freq_pub;
std_msgs__msg__Float32MultiArray freq_msg;
float freq_data[4];

geometry_msgs__msg__TransformStamped tf_msg;
rcl_publisher_t tf_pub;

float x = 0.0, y = 0.0, theta = 0.0;

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

  float delta_x = vx * cos(theta) * dt - vy * sin(theta) * dt;
  float delta_y = vx * sin(theta) * dt + vy * cos(theta) * dt;
  float delta_theta = omega * dt;

  x += delta_x;
  y += delta_y;
  theta += delta_theta;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  geometry_msgs__msg__Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(theta / 2.0);
  q.w = cos(theta / 2.0);
  odom_msg.pose.pose.orientation = q;


  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;

  rcl_publish(&odom_pub, &odom_msg, NULL);

  freq_msg.data.data = freq_data;
  freq_msg.data.size = 4;
  rcl_publish(&freq_pub, &freq_msg, NULL);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  for (int i = 0; i < 4; i++) {
    pinMode(encoder_pins[i], INPUT);
    setup_pcnt_encoder(i);
  }

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"
  );

  rclc_publisher_init_default(
    &freq_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_freq"
  );

  freq_msg.data.data = freq_data;
  freq_msg.data.size = 4;
  freq_msg.data.capacity = 4;

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),  // 100ms
    compute_and_publish_odom
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}