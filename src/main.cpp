#include <Arduino.h>

#define PUL1_PIN 18   // Tín hiệu xung cho động cơ 1
#define PUL2_PIN 14   // Tín hiệu xung cho động cơ 2
#define PUL3_PIN 32   // Tín hiệu xung cho động cơ 3
#define PUL4_PIN 25   // Tín hiệu xung cho động cơ 4
#define DIR1_PIN 19   // Tín hiệu chiều quay cho động cơ 1
#define DIR2_PIN 27   // Tín hiệu chiều quay cho động cơ 2
#define DIR3_PIN 33   // Tín hiệu chiều quay cho động cơ 3
#define DIR4_PIN 26   // Tín hiệu chiều quay cho động cơ 4

#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_CHANNEL_3 2
#define PWM_CHANNEL_4 3

float max_duty_cycle = 0.5; // Tỷ lệ chu kỳ xung tối đa (50%) (default value)


// Hàm xử lý Frequency và Duty Cycle
void ProcessFrequency(long freq, float duty_cycle, int PWM_PIN, int PWM_CHANNEL, int PWM_RESOLUTION) {
  int PWM_FREQ = freq;       // Tần số PWM
  float current_duty_percent = duty_cycle;  // Duty cycle (tỷ lệ xung)
  float max_duty_cycle;
  
  // Cập nhật PWM Resolution dựa trên tần số
  if (PWM_FREQ >= 500000) {      // 500kHz - 1MHz: 6-bit
    PWM_RESOLUTION = 6;
  } else if (PWM_FREQ >= 250000) { // 250-500kHz: 7-bit
    PWM_RESOLUTION = 7;
  } else if (PWM_FREQ >= 125000) { // 125-250kHz: 8-bit
    PWM_RESOLUTION = 8;
  } else if (PWM_FREQ >= 60000) {  // 60-125kHz: 9-bit
    PWM_RESOLUTION = 9;
  } else {                        // <60kHz: 10-bit
    PWM_RESOLUTION = 10;
  }

  max_duty_cycle = (1 << PWM_RESOLUTION) - 1;
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Cấu hình PWM với tần số và độ phân giải
  ledcAttachPin(PWM_PIN, PWM_CHANNEL); // Gắn pin với kênh PWM

  // Tính và thiết lập duty cycle
  ledcWrite(PWM_CHANNEL, (current_duty_percent * max_duty_cycle) / 100.0); 

  // In trạng thái ra Serial
  String freqStr;
  if (PWM_FREQ >= 1000000) {
    freqStr = String(PWM_FREQ / 1000000.0, 1) + "MHz";
  } else if (PWM_FREQ >= 1000) {
    freqStr = String(PWM_FREQ / 1000.0, 1) + "kHz";
  } else {
    freqStr = String(PWM_FREQ) + "Hz";
  }

  Serial.print("Current Frequency is: ");
  Serial.print(freqStr);
  Serial.print(" (");
  Serial.print(PWM_RESOLUTION);
  Serial.print("-bit)");
  Serial.print("; Dutycycle: ");
  Serial.print(current_duty_percent, 1);
  Serial.println("%");
}


void SpeedRamp(long MaxFreq, long MaxFreqHoldTime, long TotalPulse, int PWM_PIN, int PWM_CHANNEL, int PWM_RESOLUTION) {
  // Tính toán thời gian để tăng tần số từ 0 đến MaxFreq
  long rampUpTime = TotalPulse / MaxFreq;  // Thời gian tăng tần số từ 0 lên MaxFreq
  
  // Tính số xung cần phát ra trong khi tăng tần số
  long pulseStep = TotalPulse / 2;  // Giả sử nửa thời gian cho việc tăng dần tần số
  
  // Bắt đầu tăng tần số từ 0 tới MaxFreq
  for (long pulseCount = 0; pulseCount < pulseStep; pulseCount++) {
    // Tính toán tần số mới từ 0 tới MaxFreq
    long currentFreq = MaxFreq * (pulseCount / float(pulseStep));  // Tăng tần số từ 0 đến MaxFreq
    
    // Gọi hàm ProcessFrequency để cập nhật tần số PWM
    ProcessFrequency(currentFreq, 50.0, PWM_PIN, PWM_CHANNEL, PWM_RESOLUTION);  // Giữ duty cycle ở mức 50%

    delay(rampUpTime / pulseStep);  // Đảm bảo tăng tần số từ từ theo từng bước
  }
  
  // Giữ tần số tối đa trong khoảng thời gian MaxFreqHoldTime
  ProcessFrequency(MaxFreq, 50.0, PWM_PIN, PWM_CHANNEL, PWM_RESOLUTION);
  delay(MaxFreqHoldTime);  // Giữ tần số ở mức MaxFreq trong MaxFreqHoldTime
  
  // Tạo hiệu ứng giảm tần số từ MaxFreq về 1
  for (long pulseCount = pulseStep; pulseCount > 1; pulseCount--) {
    // Tính toán tần số giảm dần từ MaxFreq về 1
    long currentFreq = MaxFreq * (pulseCount / float(pulseStep));  // Giảm tần số từ MaxFreq về 0
    
    // Gọi hàm ProcessFrequency để cập nhật tần số PWM
    ProcessFrequency(currentFreq, 50.0, PWM_PIN, PWM_CHANNEL, PWM_RESOLUTION);  // Giữ duty cycle ở mức 50%

    delay(rampUpTime / pulseStep);  // Đảm bảo giảm tần số từ từ theo từng bước
  }
  
  // Sau khi giảm tần số về 0, có thể thực hiện thêm các tác vụ nếu cần
  ProcessFrequency(0, 50.0, PWM_PIN, PWM_CHANNEL, PWM_RESOLUTION);
}


// Sử dụng hàm ProcessFrequency
void setup() {
  Serial.begin(115200);
  delay(200);
  
  long MaxFreq = 100000;           // Tần số tối đa 5kHz
  long MaxFreqHoldTime = 2000;   // Giữ tần số tối đa trong 2 giây
  long TotalPulse = 10000;       // Tổng số xung cần phát ra

  int PWM_RESOLUTION = 10;       // Độ phân giải 10-bit

  // Gọi hàm SpeedRamp
  SpeedRamp(MaxFreq, MaxFreqHoldTime, TotalPulse, PUL1_PIN, PWM_CHANNEL_1, PWM_RESOLUTION);
  
}

void loop() {
  // Có thể thêm các chức năng khác vào đây nếu cần
}
