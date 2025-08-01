#include <Arduino.h>
#include <DFRobot_BMX160.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>

// ==== Cấu hình chân ====
#define PUL_PINS   {18, 14, 32, 25}
#define DIR_PINS   {19, 27, 33, 26}
#define CHANNELS   {0, 1, 2, 3}

const int pwm_pins[4] = PUL_PINS;
const int dir_pins[4] = DIR_PINS;
const int channels[4] = CHANNELS;

// ==== BMX160 ====
DFRobot_BMX160 bmx160;

// ==== micro-ROS ====
rcl_node_t node;
rcl_subscription_t freq_sub;
rcl_publisher_t imu_pub;
rcl_publisher_t log_pub;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__String input_msg;
std_msgs__msg__String log_msg;
sensor_msgs__msg__Imu imu_msg;

// ==== Macro ====
#define RCCHECK(fn) { rcl_ret_t rc = fn; if ((rc != RCL_RET_OK)) while(1){delay(1000);} }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if ((rc != RCL_RET_OK)) {} }

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

// ==== Điều khiển PWM ====
void set_motor_freq_and_dir(long freqs[4], int dirs[4]) {
  char line[128];
  snprintf(line, sizeof(line),
           "PWM | M1:%ld/%d M2:%ld/%d M3:%ld/%d M4:%ld/%d",
           freqs[0], dirs[0],
           freqs[1], dirs[1],
           freqs[2], dirs[2],
           freqs[3], dirs[3]);
  publish_log("%s", line);

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

    if (freqs[i] == 0) {
      ledcWrite(channels[i], 0);  // dừng xung
    } else {
      ledcWrite(channels[i], max_duty / 2);  // 50% duty
    }

    digitalWrite(dir_pins[i], dirs[i]);
  }
}


// ==== Nhận lệnh điều khiển từ freq_cmd ====
void freq_cmd_callback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  publish_log("[CMD] %s", msg->data.data);

  long freqs[4];
  int dirs[4];

  if (sscanf(msg->data.data, "SET %ld %d %ld %d %ld %d %ld %d",
             &freqs[0], &dirs[0], &freqs[1], &dirs[1],
             &freqs[2], &dirs[2], &freqs[3], &dirs[3]) == 8) {
    set_motor_freq_and_dir(freqs, dirs);
  } else {
    publish_log("[ERROR] Invalid cmd: %s", msg->data.data);
  }
}

// ==== Timer đọc IMU và publish ====
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer == NULL) return;

  sBmx160SensorData_t accel, gyro, magn;
  bmx160.getAllData(&magn, &gyro, &accel);

  imu_msg.angular_velocity.x = gyro.x;
  imu_msg.angular_velocity.y = gyro.y;
  imu_msg.angular_velocity.z = gyro.z;

  imu_msg.linear_acceleration.x = accel.x;
  imu_msg.linear_acceleration.y = accel.y;
  imu_msg.linear_acceleration.z = accel.z;

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = -1;
    imu_msg.angular_velocity_covariance[i] = 0;
    imu_msg.linear_acceleration_covariance[i] = 0;
  }

  rcl_publish(&imu_pub, &imu_msg, NULL);
}

// ==== Setup chính ====
void setup() {
  Serial.begin(115200);
  delay(1000);

  // BMX160 init
  if (!bmx160.begin()) {
    Serial.println("❌ BMX160 init failed");
    while (1);
  }

  // micro-ROS init
  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/raw"));

  RCCHECK(rclc_publisher_init_default(
    &log_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "output_log"));

  RCCHECK(rclc_subscription_init_default(
    &freq_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "freq_cmd"));

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),  // 100ms = 10Hz
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &freq_sub, &input_msg, &freq_cmd_callback, ON_NEW_DATA));

  // Pin setup
  for (int i = 0; i < 4; i++) {
    pinMode(dir_pins[i], OUTPUT);
    digitalWrite(dir_pins[i], LOW);
  }

  input_msg.data.data = (char*)malloc(128);
  input_msg.data.capacity = 128;
  input_msg.data.size = 0;

  log_msg.data.data = (char*)malloc(128);
  log_msg.data.capacity = 128;
  log_msg.data.size = 0;

  Serial.println("[INFO] Node started");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
