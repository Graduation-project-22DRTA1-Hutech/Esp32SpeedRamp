// ===== ENCODER QUICK TEST FOR ESP32 =====
// Kiểm tra xem ESP32 có nhận xung từ RS422->TTL không
// Copy y chang, không cần thư viện nào

volatile long enc0 = 0;
volatile long enc1 = 0;
volatile long enc2 = 0;
volatile long enc3 = 0;

// ==== CHỈ THAY NẾU CHÂN MÀY KHÁC ====
// Dùng default chân tao set cho mày:
// FL = 34, FR = 35, RL = 36, RR = 39
#define ENC_FL 34
#define ENC_FR 35
#define ENC_RL 36
#define ENC_RR 39

void IRAM_ATTR isr_fl() { enc0++; }
void IRAM_ATTR isr_fr() { enc1++; }
void IRAM_ATTR isr_rl() { enc2++; }
void IRAM_ATTR isr_rr() { enc3++; }

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("===== ENCODER TEST STARTED =====");
  Serial.println("Quay robot, xem xung có nhảy không");
  Serial.println("Nếu tất cả đều 0 -> wiring sai, hoặc RS422->TTL không ra tín hiệu");

  pinMode(ENC_FL, INPUT_PULLUP);
  pinMode(ENC_FR, INPUT_PULLUP);
  pinMode(ENC_RL, INPUT_PULLUP);
  pinMode(ENC_RR, INPUT_PULLUP);

  // Dùng CHANGE vì module RS422->TTL thường toggle mức
  attachInterrupt(ENC_FL, isr_fl, CHANGE);
  attachInterrupt(ENC_FR, isr_fr, CHANGE);
  attachInterrupt(ENC_RL, isr_rl, CHANGE);
  attachInterrupt(ENC_RR, isr_rr, CHANGE);
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 300) {
    last = millis();

    Serial.print("FL: "); Serial.print(enc0);
    Serial.print("   FR: "); Serial.print(enc1);
    Serial.print("   RL: "); Serial.print(enc2);
    Serial.print("   RR: "); Serial.println(enc3);
  }
}
