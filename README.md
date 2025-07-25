
# Esp32SpeedRamp

Dự án này là bộ điều khiển tăng tốc cho ESP32, được thiết kế cho các ứng dụng robot và tự động hóa. Dự án sử dụng framework Arduino và tích hợp micro-ROS để giao tiếp nâng cao. Hỗ trợ kết nối cảm biến, bao gồm IMU DFRobot BMX160, và cung cấp nhiều ví dụ về điều khiển tăng tốc và đo quãng đường.

## Tính năng
- Hỗ trợ bo mạch phát triển ESP32
- Framework Arduino
- Tích hợp micro-ROS (phiên bản Humble, giao tiếp serial)
- Hỗ trợ cảm biến DFRobot BMX160 IMU
- Các ví dụ về điều khiển tăng tốc, đo quãng đường và giao tiếp UART

## Cấu trúc thư mục
- `src/` - Mã nguồn chính (file khởi động: `main.cpp`)
- `lib/` - Thư viện bổ sung
- `include/` - File header
- `other/` - Các ví dụ:
  - `bmx160.cpp` - Ví dụ cảm biến BMX160
  - `espSpeedRamp1.cpp` - Ví dụ tăng tốc cơ bản
  - `espSpeedRampwithOdom.cpp` - Tăng tốc kèm đo quãng đường
  - `espSpeedRampWithUart.cpp` - Tăng tốc kèm giao tiếp UART
- `test/` - File kiểm thử
- `platformio.ini` - File cấu hình PlatformIO

## Phụ thuộc
- [micro-ROS PlatformIO](https://github.com/micro-ROS/micro_ros_platformio)
- [DFRobot BMX160](https://github.com/DFRobot/DFRobot_BMX160)

## Bắt đầu
1. **Cài đặt PlatformIO**: [Hướng dẫn cài đặt PlatformIO](https://platformio.org/install)
2. **Clone kho mã nguồn này**:
   ```bash
   git clone https://github.com/thanhnhan24/Esp32SpeedRamp.git
   ```
3. **Mở dự án bằng PlatformIO/VS Code**
4. **Kết nối bo mạch ESP32**
5. **Biên dịch và nạp firmware**:
   - Sử dụng nút build/upload của PlatformIO, hoặc chạy lệnh:
     ```bash
     pio run --target upload
     ```
6. **Theo dõi output serial**:
   - Tốc độ baud mặc định: `115200`

## Cấu hình
Xem file `platformio.ini` để thiết lập bo mạch, framework và thư viện. Có thể điều chỉnh giao thức micro-ROS và phiên bản distro nếu cần.

## Giấy phép
Dự án này sử dụng giấy phép MIT.
