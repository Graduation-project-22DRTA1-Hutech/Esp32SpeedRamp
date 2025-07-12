#include <Arduino.h>

// Định nghĩa các chân PWM và DIR
#define PUL1_PIN 18
#define PUL2_PIN 14
#define PUL3_PIN 32
#define PUL4_PIN 25

#define DIR1_PIN 19
#define DIR2_PIN 27
#define DIR3_PIN 33
#define DIR4_PIN 26

#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_CHANNEL_3 2
#define PWM_CHANNEL_4 3

// Biến lưu tần số và duty cycle hiện tại
long freqs[4] = {1000, 1000, 1000, 1000};
float duties[4] = {20.0, 20.0, 20.0, 20.0};

// ======================== Hàm điều khiển PWM ==========================
void ProcessFrequency_MultiMotor(long freq[4], float duty_cycle[4]) {
  int channels[4] = {PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, PWM_CHANNEL_4};
  int pins[4] = {PUL1_PIN, PUL2_PIN, PUL3_PIN, PUL4_PIN};

  for (int i = 0; i < 4; i++) {
    int PWM_RESOLUTION;

    // Xác định độ phân giải
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

    ledcSetup(channels[i], freq[i], PWM_RESOLUTION);
    ledcAttachPin(pins[i], channels[i]);
    ledcWrite(channels[i], (duty_cycle[i] * max_duty) / 100.0);
  }

  // In trạng thái
  Serial.println("Cập nhật tần số & duty:");
  for (int i = 0; i < 4; i++) {
    Serial.print("  Motor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(freq[i]);
    Serial.print(" Hz, ");
    Serial.print(duty_cycle[i], 1);
    Serial.println(" %");
  }
}

// ======================== Setup ==========================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Thiết lập chiều quay mặc định
  pinMode(DIR1_PIN, OUTPUT); digitalWrite(DIR1_PIN, HIGH);
  pinMode(DIR2_PIN, OUTPUT); digitalWrite(DIR2_PIN, HIGH);
  pinMode(DIR3_PIN, OUTPUT); digitalWrite(DIR3_PIN, HIGH);
  pinMode(DIR4_PIN, OUTPUT); digitalWrite(DIR4_PIN, HIGH);

  // Khởi động PWM lần đầu
  ProcessFrequency_MultiMotor(freqs, duties);

  Serial.println("Nhập lệnh dạng:");
  Serial.println("  SET freq1 duty1 freq2 duty2 freq3 duty3 freq4 duty4");
  Serial.println("VD: SET 1000 20 2000 30 3000 40 4000 50");
}

// ======================== Vòng lặp chính ==========================
void loop() {
  static String inputString = "";
  static bool stringComplete = false;

  // Đọc dữ liệu serial
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // Xử lý chuỗi lệnh
  if (stringComplete) {
    inputString.trim();  // Xoá khoảng trắng đầu/cuối
    if (inputString.startsWith("SET")) {
      // Parse dữ liệu
      long tempFreqs[4];
      float tempDuties[4];
      int parsed = sscanf(inputString.c_str(), "SET %ld %f %ld %f %ld %f %ld %f",
                          &tempFreqs[0], &tempDuties[0],
                          &tempFreqs[1], &tempDuties[1],
                          &tempFreqs[2], &tempDuties[2],
                          &tempFreqs[3], &tempDuties[3]);

      if (parsed == 8) {
        // Cập nhật mảng toàn cục
        for (int i = 0; i < 4; i++) {
          freqs[i] = tempFreqs[i];
          duties[i] = tempDuties[i];
        }
        ProcessFrequency_MultiMotor(freqs, duties);
      } else {
        Serial.println("❌ Lỗi: Sai cú pháp! Nhập đủ 8 tham số sau SET");
      }
    } else {
      Serial.println("❌ Lệnh không hợp lệ. Dùng: SET freq1 duty1 freq2 duty2 freq3 duty3 freq4 duty4");
    }

    inputString = "";
    stringComplete = false;
  }
}
