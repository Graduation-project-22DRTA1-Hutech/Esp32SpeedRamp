#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino with serial transport.
#endif

// ==== Định nghĩa chân ====
#define PUL_PINS {18, 14, 32, 25}
#define DIR_PINS {19, 27, 33, 26}
#define CHANNELS {0, 1, 2, 3}

// ==== Biến PWM ====
long freqs[4] = {1000, 1000, 1000, 1000};
float duties[4] = {20.0, 20.0, 20.0, 20.0};

// ==== micro-ROS ====
rcl_subscription_t subscriber;
std_msgs__msg__String msg;

rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// ==== Macro xử lý lỗi ====
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop() {
  while (1) {
    delay(100);
  }
}

// ==== Hàm điều khiển PWM ====
void ProcessFrequency_MultiMotor(long freq[4], float duty_cycle[4]) {
  int pins[4] = PUL_PINS;
  int channels[4] = CHANNELS;

  for (int i = 0; i < 4; i++) {
    int PWM_RESOLUTION;
    if (freq[i] >= 500000) PWM_RESOLUTION = 6;
    else if (freq[i] >= 250000) PWM_RESOLUTION = 7;
    else if (freq[i] >= 125000) PWM_RESOLUTION = 8;
    else if (freq[i] >= 60000)  PWM_RESOLUTION = 9;
    else PWM_RESOLUTION = 10;

    int max_duty = (1 << PWM_RESOLUTION) - 1;
    ledcSetup(channels[i], freq[i], PWM_RESOLUTION);
    ledcAttachPin(pins[i], channels[i]);
    ledcWrite(channels[i], (duty_cycle[i] * max_duty) / 100.0);
  }
}

// ==== Callback khi nhận chuỗi ROS ====
void string_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  String inputString = String(msg->data.data);  // Chuyển const char* sang String
  inputString.trim();                          // Gọi trim() sau đó


  if (inputString.startsWith("SET")) {
    inputString.remove(0, 3);
    inputString.trim();

    // SET dir-freq
    if (inputString.indexOf('-') != -1) {
      int dir = inputString.substring(0, inputString.indexOf('-')).toInt();
      long freq = inputString.substring(inputString.indexOf('-') + 1).toInt();

      if ((dir == 0 || dir == 1) && freq > 0) {
        int dirPins[4] = DIR_PINS;
        for (int i = 0; i < 4; i++) {
          digitalWrite(dirPins[i], dir);
          freqs[i] = freq;
          duties[i] = 50.0;
        }
        ProcessFrequency_MultiMotor(freqs, duties);
      }

    } else {
      // SET freq1 duty1 freq2 duty2 freq3 duty3 freq4 duty4
      long tempFreqs[4];
      float tempDuties[4];
      int parsed = sscanf(inputString.c_str(), "%ld %f %ld %f %ld %f %ld %f",
                          &tempFreqs[0], &tempDuties[0],
                          &tempFreqs[1], &tempDuties[1],
                          &tempFreqs[2], &tempDuties[2],
                          &tempFreqs[3], &tempDuties[3]);
      if (parsed == 8) {
        for (int i = 0; i < 4; i++) {
          freqs[i] = tempFreqs[i];
          duties[i] = tempDuties[i];
        }
        ProcessFrequency_MultiMotor(freqs, duties);
      }
    }
  }
}

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  delay(2000);
  set_microros_serial_transports(Serial);

  // Cấu hình chân
  int dirPins[4] = DIR_PINS;
  for (int i = 0; i < 4; i++) {
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], HIGH);
  }

  ProcessFrequency_MultiMotor(freqs, duties);

  allocator = rcl_get_default_allocator();

  // Init ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "mros_freq_node", "", &support));

  // Init subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "freq_cmd"
  ));

  // Init executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &string_callback, ON_NEW_DATA));
}

// ==== Vòng lặp chính ====
void loop() {
  delay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
