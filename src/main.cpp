#include <Arduino.h>
#include <Wire.h>

#define I2C_SLAVE_ADDR 0x99
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("ESP32 Master bắt đầu...");
}

void loop() {
  Wire.requestFrom(I2C_SLAVE_ADDR, 32);  // Yêu cầu 32 byte

  if (Wire.available()) {
    Serial.print("Nhận từ slave: ");
    while (Wire.available()) {
      char c = Wire.read();
      Serial.print(c);
    }
    Serial.println();
  } else {
    Serial.println("Không nhận được dữ liệu từ slave");
  }

  delay(1000);  // 1 giây mỗi lần yêu cầu
}
