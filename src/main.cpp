#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example requires Arduino serial transport for micro-ROS
#endif

// micro-ROS core
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__String input_msg;
std_msgs__msg__String output_msg;

// Trạng thái theo dõi
char feedback_buffer[64];
volatile bool command_received = false;

// LED_BUILTIN (GPIO2)
const int led_pin = 2;

// ========== Hàm lỗi ==========
void error_loop() {
  while (1) {
    Serial.println("[ERROR] micro-ROS init failed.");
    delay(1000);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {}}

// ========== Xử lý lệnh ==========
void parse_and_execute(const char *data) {
  if (strncmp(data, "SET ", 4) != 0) {
    snprintf(feedback_buffer, sizeof(feedback_buffer), "[ERROR] Invalid format: %s", data);
    return;
  }

  int pin = -1, state = -1;
  if (sscanf(data + 4, "%d-%d", &pin, &state) == 2) {
    if (pin >= 0 && pin <= 39 && (state == 0 || state == 1)) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, state);
      snprintf(feedback_buffer, sizeof(feedback_buffer), "[LOG] DONE SET %d-%d", pin, state);
      Serial.printf("[INFO] GPIO %d set to %d\n", pin, state);
    } else {
      snprintf(feedback_buffer, sizeof(feedback_buffer), "[ERROR] Invalid pin/state: %d-%d", pin, state);
    }
  } else {
    snprintf(feedback_buffer, sizeof(feedback_buffer), "[ERROR] Parse error: %s", data);
  }
}

// ========== Callback subscriber ==========
void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  Serial.println("[DEBUG] subscription_callback called");
  digitalWrite(led_pin, !digitalRead(led_pin));  // Toggle LED

  command_received = true;

  Serial.printf("[INFO] Received: %s\n", msg->data.data);

  parse_and_execute(msg->data.data);

  strncpy(output_msg.data.data, feedback_buffer, output_msg.data.capacity - 1);
  output_msg.data.data[output_msg.data.capacity - 1] = '\0';
  output_msg.data.size = strlen(output_msg.data.data);

  rcl_publish(&publisher, &output_msg, NULL);
  Serial.printf("[INFO] Sent to output_cmd: %s\n", output_msg.data.data);
}

// ========== Callback timer ==========
void timer_callback(rcl_timer_t *, int64_t) {
  if (!command_received) {
    const char *waiting_msg = "[INFO] waiting for command";
    strncpy(output_msg.data.data, waiting_msg, output_msg.data.capacity - 1);
    output_msg.data.data[output_msg.data.capacity - 1] = '\0';
    output_msg.data.size = strlen(output_msg.data.data);

    rcl_publish(&publisher, &output_msg, NULL);
    Serial.println("[TIMER] waiting for command (no new data)");
  }
  command_received = false;
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  pinMode(led_pin, OUTPUT);  // LED để debug nháy khi nhận lệnh

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_gpio_node", "", &support));

  // Subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "input_cmd"));

  // Publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "output_cmd"));

  // ✅ Cấp phát bộ nhớ
  input_msg.data.data = (char *)malloc(64);
  input_msg.data.size = 0;
  input_msg.data.capacity = 64;

  output_msg.data.data = (char *)malloc(64);
  output_msg.data.size = 0;
  output_msg.data.capacity = 64;

  // Timer 5 giây
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(5000),
    timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &input_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("[INFO] Node initialized. Waiting for input...");
}

// ========== Loop ==========
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
