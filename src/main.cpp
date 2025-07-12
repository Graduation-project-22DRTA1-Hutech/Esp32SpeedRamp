#include <Arduino.h>

// Định nghĩa các chân PWM và DIR cho từng động cơ
#define PUL1_PIN 18
#define PUL2_PIN 14
#define PUL3_PIN 32
#define PUL4_PIN 25

#define DIR1_PIN 19
#define DIR2_PIN 27
#define DIR3_PIN 33
#define DIR4_PIN 26

// Định nghĩa các kênh PWM
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_CHANNEL_3 2
#define PWM_CHANNEL_4 3

// Hàm xử lý PWM cho 4 động cơ với tần số và duty cycle riêng
void ProcessFrequency_MultiMotor(long freq[4], float duty_cycle[4]) {
  int channels[4] = {PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, PWM_CHANNEL_4};
  int pins[4] = {PUL1_PIN, PUL2_PIN, PUL3_PIN, PUL4_PIN};

  for (int i = 0; i < 4; i++) {
    int PWM_RESOLUTION;

    // Xác định độ phân giải theo tần số từng động cơ
    if (freq[i] >= 500000)
      PWM_RESOLUTION = 6;
    else if (freq[i] >= 250000)
      PWM_RESOLUTION = 7;
    else if (freq[i] >= 125000)
      PWM_RESOLUTION = 8;
    else if (freq[i] >= 60000)
      PWM_RESOLUTION = 9;
    else
      PWM_RESOLUTION = 10;

    int max_duty = (1 << PWM_RESOLUTION) - 1;

    ledcSetup(channels[i], freq[i], PWM_RESOLUTION); // Thiết lập tần số và độ phân giải
    ledcAttachPin(pins[i], channels[i]);              // Gắn pin PWM
    ledcWrite(channels[i], (duty_cycle[i] * max_duty) / 100.0); // Viết duty cycle
  }

  // In trạng thái ra Serial
  for (int i = 0; i < 4; i++) {
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" - Freq: ");
    Serial.print(freq[i]);
    Serial.print(" Hz; Duty: ");
    Serial.print(duty_cycle[i], 1);
    Serial.println("%");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Thiết lập chiều quay (có thể tùy chỉnh HIGH/LOW tuỳ yêu cầu)
  pinMode(DIR1_PIN, OUTPUT); digitalWrite(DIR1_PIN, HIGH);
  pinMode(DIR2_PIN, OUTPUT); digitalWrite(DIR2_PIN, HIGH);
  pinMode(DIR3_PIN, OUTPUT); digitalWrite(DIR3_PIN, HIGH);
  pinMode(DIR4_PIN, OUTPUT); digitalWrite(DIR4_PIN, HIGH);

  // Thiết lập tần số và duty cycle riêng cho từng động cơ
  long freqs[4] = {1000, 2000, 3000, 4000};     // Tần số (Hz)
  float duties[4] = {20.0, 30.0, 50.0, 70.0};   // Duty cycle (%)

  // Gọi hàm điều khiển 4 động cơ đồng thời
  ProcessFrequency_MultiMotor(freqs, duties);
}

void loop() {
  // Để trống hoặc thêm logic điều khiển khác tại đây
}
