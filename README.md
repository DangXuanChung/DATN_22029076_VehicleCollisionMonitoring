# DATN_22029076_VehicleCollisionMonitoring
Đồ án tốt nghiệp – Hệ thống định vị và giám sát va chạm ô tô thời gian thực
# Hệ thống Định vị và Giám sát Va chạm Ô tô Thời gian Thực
> Đồ án tốt nghiệp đại học hệ chính quy  
## 1. Giới thiệu

Đồ án xây dựng một thiết bị nhúng IoT có khả năng định vị xe theo thời gian thực qua GPS/4G và tự động phát hiện va chạm hoặc lật xe thông qua cảm biến quán tính, đồng thời gửi tín hiệu SOS kèm tọa độ tới giao diện web giám sát. Hệ thống được thiết kế nhằm rút ngắn thời gian ứng cứu sau tai nạn trong khung "giờ vàng" y tế.

## 2. Phần cứng sử dụng

| Thành phần | Mô tả |
|---|---|
| Vi điều khiển | ESP32-WROOM-32E NodeMCU 38 chân (dual-core Xtensa LX6, 240 MHz) |
| Cảm biến quán tính | MPU6050 (gia tốc kế 3 trục ±16g + con quay hồi chuyển 3 trục ±2000°/s) |
| Module 4G/GPS | Quectel EG800K trên bo TDM2404 (LTE Cat-1 + GNSS đa hệ thống) |
| Mạch nguồn | Mạch hạ áp xung MP1584EN |
| Nguồn cấp | Adapter 12V DC |

## 3. Cấu trúc dự án

```
.
├── docs/              Tài liệu đồ án (.pdf)
├── firmware/          Mã nguồn firmware ESP32 (PlatformIO)
│   ├── src/main.cpp
│   └── platformio.ini
├── web/               Giao diện web giám sát
│   └── index.html
└── hardware/          Sơ đồ phần cứng
```

## 4. Tính năng chính

Hệ thống định vị GPS thời gian thực với độ chính xác từ 3 đến 5 mét trong điều kiện trống thoáng. Phát hiện va chạm theo ngưỡng gia tốc tổng hợp 8.0G và phát hiện lật xe theo ngưỡng góc nghiêng 60°, có xác minh hai lớp gồm cửa sổ 2 giây và đếm ngược 10 giây cho phép người dùng hủy SOS. Bộ lọc Kalman được triển khai phần mềm để ước lượng góc Pitch và Roll từ dữ liệu IMU. Truyền tin qua giao thức MQTT trên mạng 4G LTE với cờ Retain để bảo đảm khôi phục trạng thái khi giao diện web kết nối lại sau gián đoạn. Giao diện web hiển thị vị trí trên bản đồ Leaflet.js, có cơ chế cache LocalStorage cho trường hợp mất kết nối.

## 5. Build firmware

```bash
# Cài đặt PlatformIO Core
pip install platformio

# Vào thư mục firmware, compile và upload qua USB
cd firmware
pio run -t upload

# Mở Serial Monitor
pio device monitor -b 115200
```

Trước khi build, kiểm tra `platformio.ini` đã chọn đúng board `esp32dev` và cổng COM phù hợp. Nếu không upload được, kiểm tra driver CP210x hoặc CH340 trên Windows.

## 6. Chạy giao diện web

Mở file `web/index.html` trực tiếp bằng trình duyệt bất kỳ (Chrome, Firefox, Edge). Không cần web server riêng. Giao diện sẽ tự động kết nối tới broker MQTT công cộng tại `broker.hivemq.com:8000` qua WebSocket và đăng ký vào topic được khai báo sẵn trong file HTML.

## 7. Kết quả thực nghiệm tóm tắt

| Tiêu chí | Giá trị đo |
|---|---|
| Độ chính xác GPS | 21.16585°N, 106.05692°E (sai số ≤ 5m) |
| Phát hiện va chạm tốc độ thấp | 8.42G tại 10 km/h → phát hiện thành công |
| Phát hiện lật xe | 68°–71° → phát hiện thành công |
| Tỷ lệ cảnh báo giả | 0/10 trên các tình huống vận hành thường |
| Độ trễ E2E trung bình | 565 ms (ESP32 → web giám sát) |

Chi tiết kết quả trình bày trong Chương 4 của đồ án.

## 8. Phiên bản nộp Hội đồng

Phiên bản chính thức nộp đồ án tương ứng với release tag `v1.0-thesis-submission`. Commit hash đầy đủ được ghi cụ thể trong Phụ lục của tài liệu đồ án.

## 9. Tài liệu đồ án
????
## 10. Giấy phép

Mã nguồn được cấp phép theo MIT License cho mục đích học thuật và phi thương mại. Vui lòng trích dẫn nếu sử dụng cho công trình nghiên cứu khác.
