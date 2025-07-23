#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ====== Encoder ISR ======
volatile long encoder_counts[4] = {0, 0, 0, 0};
volatile long prev_counts[4] = {0, 0, 0, 0};
void IRAM_ATTR encoder_ISR_0() { encoder_counts[0]++; }
void IRAM_ATTR encoder_ISR_1() { encoder_counts[1]++; }
void IRAM_ATTR encoder_ISR_2() { encoder_counts[2]++; }
void IRAM_ATTR encoder_ISR_3() { encoder_counts[3]++; }

// ====== Define pins ======
#define PUL_PINS   {18, 14, 32, 25}
#define DIR_PINS   {19, 27, 33, 26}
#define CHANNELS   {0, 1, 2, 3}
#define ENCODER_PINS {34, 35, 36, 39}

#define PPR 5000
#define WHEEL_RADIUS 0.05  // 100mm diameter => 50mm radius
#define LX 0.12
#define LY 0.12

// ====== micro-ROS node setup ======
rcl_node_t node;
rcl_subscription_t freq_sub;
rcl_publisher_t log_pub;
rcl_publisher_t odom_pub;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__String input_msg;
std_msgs__msg__String log_msg;
nav_msgs__msg__Odometry odom_msg;

float x = 0, y = 0, theta = 0;
unsigned long last_odom_time = 0;

void error_loop() {
  while (1) delay(1000);
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {}}

// ====== Log helper ======
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

// ====== PWM & DIR control ======
void set_motor_freq_and_dir(long freqs[4], int dirs[4]) {
  const int pwm_pins[4] = PUL_PINS;
  const int dir_pins[4] = DIR_PINS;
  const int channels[4] = CHANNELS;

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

// ====== Callback nhận lệnh ======
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

// ====== Tính odom từ encoder ======
void compute_and_publish_odom() {
  unsigned long now = millis();
  float dt = (now - last_odom_time) / 1000.0;
  if (dt < 0.01) return;
  last_odom_time = now;

  float v_wheel[4];
  for (int i = 0; i < 4; i++) {
    long delta = encoder_counts[i] - prev_counts[i];
    prev_counts[i] = encoder_counts[i];

    float revolutions = (float)delta / PPR;
    v_wheel[i] = (revolutions * 2 * PI * WHEEL_RADIUS) / dt; // m/s
  }

  float vx = (v_wheel[0] + v_wheel[1] + v_wheel[2] + v_wheel[3]) / 4.0;
  float vy = (-v_wheel[0] + v_wheel[1] + v_wheel[2] - v_wheel[3]) / 4.0;
  float omega = (-v_wheel[0] + v_wheel[1] - v_wheel[2] + v_wheel[3]) / (4 * (LX + LY));

  // Update position
  float dx = vx * dt * cos(theta) - vy * dt * sin(theta);
  float dy = vx * dt * sin(theta) + vy * dt * cos(theta);
  float dtheta = omega * dt;

  x += dx;
  y += dy;
  theta += dtheta;

  // Xuất odom
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  odom_msg.pose.pose.orientation = tf2::toMsg(q);

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = omega;

  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  const int dir_pins[4] = DIR_PINS;
  for (int i = 0; i < 4; i++) {
    pinMode(dir_pins[i], OUTPUT);
    digitalWrite(dir_pins[i], LOW);
  }

  const int encoder_pins[4] = ENCODER_PINS;
  for (int i = 0; i < 4; i++) {
    pinMode(encoder_pins[i], INPUT);
  }
  attachInterrupt(encoder_pins[0], encoder_ISR_0, RISING);
  attachInterrupt(encoder_pins[1], encoder_ISR_1, RISING);
  attachInterrupt(encoder_pins[2], encoder_ISR_2, RISING);
  attachInterrupt(encoder_pins[3], encoder_ISR_3, RISING);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &freq_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "freq_cmd"));

  RCCHECK(rclc_publisher_init_default(
    &log_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "output_log"));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  input_msg.data.data = (char*)malloc(128);
  input_msg.data.size = 0;
  input_msg.data.capacity = 128;

  log_msg.data.data = (char*)malloc(128);
  log_msg.data.size = 0;
  log_msg.data.capacity = 128;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &freq_sub, &input_msg, &freq_cmd_callback, ON_NEW_DATA));

  last_odom_time = millis();
  publish_log("[INFO] micro-ROS motor node ready.");
}

// ====== Loop ======
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  compute_and_publish_odom();
}
