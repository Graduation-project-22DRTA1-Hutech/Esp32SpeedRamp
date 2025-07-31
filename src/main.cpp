#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include "DFRobot_BMX160.h"

// ==== Pin cấu hình ====
#define PUL_PINS   {18, 14, 32, 25}
#define DIR_PINS   {19, 27, 33, 26}
#define CHANNELS   {0, 1, 2, 3}

const int pwm_pins[4] = PUL_PINS;
const int dir_pins[4] = DIR_PINS;
const int channels[4] = CHANNELS;

// ==== Thông số robot ====
float freq_data[4];

// ==== micro-ROS ====
rcl_node_t node;
rcl_subscription_t freq_sub;
rcl_publisher_t log_pub;
rcl_publisher_t freq_pub;
rcl_publisher_t imu_pub;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__String input_msg;
std_msgs__msg__String log_msg;
std_msgs__msg__Float32MultiArray freq_msg;
sensor_msgs__msg__Imu imu_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) while(1){delay(1000);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

// ==== BMX160 ====
DFRobot_BMX160 bmx160;

// ==== Gửi log ====
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

// ==== Điều khiển motor ====
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
    for (int i = 0; i < 4; i++) freq_data[i] = freqs[i] * dirs[i];
  } else {
    publish_log("[ERROR] Invalid format: %s", msg->data.data);
  }
}

// ==== Đọc và gửi dữ liệu IMU ====
void read_and_publish_imu(rcl_timer_t * timer, int64_t last_call_time) {
  sBmx160SensorData_t gyro, accel, magn;
  bmx160.getAllData(&magn, &gyro, &accel);

  imu_msg.linear_acceleration.x = accel.x;
  imu_msg.linear_acceleration.y = accel.y;
  imu_msg.linear_acceleration.z = accel.z;

  imu_msg.angular_velocity.x = gyro.x;
  imu_msg.angular_velocity.y = gyro.y;
  imu_msg.angular_velocity.z = gyro.z;

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  rcl_publish(&imu_pub, &imu_msg, NULL);

  // Optional: gửi lại freq để debug
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
  }

  if (!bmx160.begin()) {
    Serial.println("IMU init failed"); while (1);
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "imu_motor_node", "", &support));

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
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"));

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
    read_and_publish_imu
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &freq_sub, &input_msg, &freq_cmd_callback, ON_NEW_DATA));

  publish_log("[INFO] Node ready.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
