#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ==========================================
// 1. CẤU HÌNH PHẦN CỨNG
// ==========================================
#define RXD2 16 
#define TXD2 17 
#define PEN_PIN 13  
#define CANCEL_BUTTON_PIN 0 // Nút BOOT
const int MPU_ADDR = 0x68;
HardwareSerial module4G(2);

// ==========================================
// 2. CẤU HÌNH THUẬT TOÁN & MQTT
// ==========================================
const float CRASH_G_THRESHOLD = 8; 
const float ROLLOVER_ANGLE = 60.0;   

const char* apn = "v-internet"; 
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "my_esp32_project/sos_system_123"; 

// ==========================================
// 3. BIẾN TOÀN CỤC CHO RTOS
// ==========================================
TaskHandle_t ModemTaskHandle;
TaskHandle_t SensorTaskHandle;

enum SystemState { STATE_IDLE, STATE_VERIFYING, STATE_WAIT_USER_CANCEL, STATE_CRASHED };
volatile SystemState currentState = STATE_IDLE; 

volatile float total_G = 0, angle_pitch = 0, angle_roll = 0;
volatile bool trigger_sos_mqtt = false; 
volatile bool trigger_power_down = false;

// ================= CLASS BỘ LỌC KALMAN =================
class KalmanFilter {
  public:
    KalmanFilter() {
      Q_angle = 0.001f; Q_bias = 0.003f; R_measure = 0.03f;
      angle = 0.0f; bias = 0.0f; 
      P[0][0] = 0.0f; P[0][1] = 0.0f; P[1][0] = 0.0f; P[1][1] = 0.0f;
    }
    float getAngle(float newAngle, float newRate, float dt) {
      rate = newRate - bias; angle += dt * rate;
      P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle); P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1]; P[1][1] += Q_bias * dt;
      float S = P[0][0] + R_measure; float K[2]; K[0] = P[0][0] / S; K[1] = P[1][0] / S;
      float y = newAngle - angle; angle += K[0] * y; bias += K[1] * y;
      float P00_temp = P[0][0]; float P01_temp = P[0][1];
      P[0][0] -= K[0] * P00_temp; P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp; P[1][1] -= K[1] * P01_temp;
      return angle;
    }
  private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias, rate;
    float P[2][2];
};

KalmanFilter kalmanPitch;
KalmanFilter kalmanRoll;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

void calibrateMPU() {
  Serial.println("HIỆU CHUẨN GYRO... ĐỂ YÊN MẠCH!");
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43);
    Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 6, true);
    sumX += (Wire.read() << 8 | Wire.read()); sumY += (Wire.read() << 8 | Wire.read()); sumZ += (Wire.read() << 8 | Wire.read());
    delay(3);
  }
  gyroX_offset = (float)sumX / 1000; gyroY_offset = (float)sumY / 1000; gyroZ_offset = (float)sumZ / 1000;
  Serial.println("Xong!");
}

String sendAT(String command, int timeout) {
  String response = "";
  module4G.println(command);
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (module4G.available()) { response += (char)module4G.read(); }
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
  Serial.print(response);
  return response;
}

void publishMQTT_AT(String topic, String payload) {
  Serial.println("\n>>> ĐANG GỬI GÓI TIN MQTT QUA 4G...");
  unsigned long t0 = millis();

  String cmd = "AT+QMTPUB=0,0,0,1,\"" + topic + "\"";
  module4G.println(cmd);
  vTaskDelay(pdMS_TO_TICKS(100));
  module4G.print(payload);
  module4G.write(26);

  // Chờ phản hồi, thoát ngay khi thấy OK — không chờ cứng 3s nữa
  String response = "";
  unsigned long deadline = millis() + 5000;
  while (millis() < deadline) {
    while (module4G.available()) {
      response += (char)module4G.read();
    }
    if (response.indexOf("+QMTPUB:") != -1 || response.indexOf("OK") != -1) {
      break; // ← Thoát ngay khi module xác nhận xong
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  Serial.print(response);
  Serial.printf("[LATENCY] ESP32→Broker: %lu ms\n", millis() - t0);
}

// ==========================================
// TASK 1: ĐỌC CẢM BIẾN & XỬ LÝ NÚT BẤM (Core 1)
// ==========================================
void TaskCoreLogic(void *pvParameters) {
  unsigned long last_time = micros();
  unsigned long state_timer = 0;
  int countdown = 10;
  unsigned long warmup_start = millis(); 
  unsigned long button_press_start = 0; // Biến đếm thời gian giữ nút

  for(;;) { 
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
    Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 14, true);
    int16_t AcX = Wire.read() << 8 | Wire.read(); int16_t AcY = Wire.read() << 8 | Wire.read(); int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); 
    int16_t GyX = Wire.read() << 8 | Wire.read(); int16_t GyY = Wire.read() << 8 | Wire.read(); int16_t GyZ = Wire.read() << 8 | Wire.read();

    unsigned long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0;
    last_time = current_time;

    float ax = AcX / 2048.0; float ay = AcY / 2048.0; float az = AcZ / 2048.0;
    total_G = sqrt(ax*ax + ay*ay + az*az);

    float gyro_rate_pitch = (GyX - gyroX_offset) / 131.0; float gyro_rate_roll  = (GyY - gyroY_offset) / 131.0;
    float accel_pitch = atan2(ay, sqrt(ax*ax + az*az)) * 180 / PI; float accel_roll  = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;

    angle_pitch = kalmanPitch.getAngle(accel_pitch, gyro_rate_pitch, dt);
    angle_roll  = kalmanRoll.getAngle(accel_roll, gyro_rate_roll, dt);

    // XỬ LÝ LOGIC NÚT BẤM KÉP (Nhấn nhả để hủy SOS / Giữ 5s để tắt mạch)
    bool isCanceled = false;
    if (digitalRead(CANCEL_BUTTON_PIN) == LOW) {
      if (button_press_start == 0) button_press_start = millis(); // Bắt đầu bấm giờ
      
      // Nếu giữ vượt quá 5 giây (5000ms)
      if (millis() - button_press_start >= 5000 && !trigger_power_down) {
        Serial.println("\n[HỆ THỐNG] Đã giữ nút 5s! Ra lệnh TẮT MẠCH AN TOÀN...");
        trigger_power_down = true; 
      }
      isCanceled = true; 
    } else {
      button_press_start = 0; // Reset bộ đếm khi nhả nút
    }

    // Bỏ qua 3s đầu tiên khởi động cảm biến
    if (millis() - warmup_start < 3000) {
      vTaskDelay(pdMS_TO_TICKS(10)); continue; 
    }

    // MÁY TRẠNG THÁI (State Machine)
    switch (currentState) {
      case STATE_IDLE:
        if (total_G > CRASH_G_THRESHOLD || abs(angle_pitch) > ROLLOVER_ANGLE || abs(angle_roll) > ROLLOVER_ANGLE) {
          Serial.println("[CẢNH BÁO] Phát hiện bất thường!");
          currentState = STATE_VERIFYING;
          state_timer = millis();
        }
        break;

      case STATE_VERIFYING:
        if (millis() - state_timer > 2000) {
          if ((total_G > 0.8 && total_G < 1.2) || abs(angle_pitch) > ROLLOVER_ANGLE || abs(angle_roll) > ROLLOVER_ANGLE) {
            Serial.println("[XÁC NHẬN TAI NẠN] Chờ Hủy...");
            currentState = STATE_WAIT_USER_CANCEL;
            state_timer = millis();
            countdown = 10;
          } else {
            currentState = STATE_IDLE; 
          }
        }
        break;

      case STATE_WAIT_USER_CANCEL:
        if (isCanceled && !trigger_power_down) { // Chỉ hủy SOS nếu không phải đang muốn tắt nguồn
          Serial.println("[HỦY] Người dùng đã bấm nút. Trở về an toàn.");
          while(digitalRead(CANCEL_BUTTON_PIN) == LOW) { vTaskDelay(pdMS_TO_TICKS(10)); } 
          currentState = STATE_IDLE;
        } 
        else if (millis() - state_timer > 10000) {
          Serial.println(">>> PHÁT LỆNH SOS <<<");
          currentState = STATE_CRASHED;
          trigger_sos_mqtt = true; 
        } 
        else {
          static unsigned long last_print = 0;
          if (millis() - last_print > 1000) {
            countdown = 10 - ((millis() - state_timer) / 1000);
            Serial.printf("SOS sẽ gửi sau %d s (BẤM BOOT ĐỂ HỦY)...\n", countdown);
            last_print = millis();
          }
        }
        break;

      case STATE_CRASHED:
        if (isCanceled && !trigger_power_down) {
          Serial.println("Hệ thống được Reset về bình thường.");
          while(digitalRead(CANCEL_BUTTON_PIN) == LOW) { vTaskDelay(pdMS_TO_TICKS(10)); }
          currentState = STATE_IDLE;
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

// ==========================================
// TASK 2: QUẢN LÝ 4G, GPS VÀ MQTT (Core 0)
// ==========================================
void ModemTask(void *pvParameters) {
  Serial.println("\n[MODEM TASK] Bắt đầu khởi tạo...");
  
  while(module4G.available()) module4G.read();
  sendAT("AT", 1000); 
  
  if (sendAT("AT", 1000).indexOf("OK") == -1) {
    Serial.println("[MODEM TASK] Kích nguồn Module 4G...");
    pinMode(PEN_PIN, OUTPUT); digitalWrite(PEN_PIN, LOW);   
    vTaskDelay(pdMS_TO_TICKS(2000)); pinMode(PEN_PIN, INPUT);      
    vTaskDelay(pdMS_TO_TICKS(10000)); 
  } 
  
  // BẮT BUỘC: Đánh thức Module 4G thoát khỏi chế độ máy bay (nếu vừa ngủ dậy)
  Serial.println("[MODEM TASK] Kích hoạt sóng RF (Thoát Flight Mode)...");
  sendAT("AT+CFUN=1", 5000);
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  sendAT("ATE0", 1000); 
  sendAT("AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"", 1000);
  sendAT("AT+QIACT=1", 4000); 
  
  if (sendAT("AT+QGPS?", 1000).indexOf("+QGPS: 1") == -1) {
    sendAT("AT+QGPS=1", 2000);
  }

  sendAT("AT+QMTCFG=\"pdpcid\",0,1", 1000); 
  sendAT("AT+QMTOPEN=0,\"" + String(mqtt_server) + "\"," + String(mqtt_port), 5000);
  vTaskDelay(pdMS_TO_TICKS(2000));
  sendAT("AT+QMTCONN=0,\"ESP32_SOS_" + String(random(0xffff), HEX) + "\"", 5000);

  String last_lat = "";
  String last_lon = "";
  bool has_first_fix = false;
  unsigned long last_gps_poll = 0;
  
  const unsigned long POLL_INTERVAL_NO_FIX = 3000;  
  const unsigned long POLL_INTERVAL_FIXED = 60000;  

  for (;;) {
    
    // ========================================================
    // KIỂM TRA LỆNH TẮT NGUỒN TỪ NGƯỜI DÙNG CHỐT XUỐNG
    // ========================================================
    if (trigger_power_down) {
      Serial.println("\n[MODEM TASK] ĐANG TIẾN HÀNH CHU TRÌNH TẮT AN TOÀN...");
      
      // Gửi bản tin Offline lên giao diện Web
      String t_lat = (last_lat != "") ? last_lat : "\"UNKNOWN\"";
      String t_lon = (last_lon != "") ? last_lon : "\"UNKNOWN\"";
      String payload = "{\"lat\":" + t_lat + 
                       ", \"lng\":" + t_lon + 
                       ", \"g_force\":0.0, \"pitch\":0.0, \"roll\":0.0, \"status\":\"OFFLINE\"}";
      publishMQTT_AT(mqtt_topic, payload);
      
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // 1. Gửi lệnh tắt bằng phần mềm trước cho an toàn dữ liệu
      Serial.println("-> Gửi lệnh AT tắt nguồn...");
      sendAT("AT+QPOWD=1", 2000); 
      
      // 2. Tắt triệt để bằng phần cứng (Ép tắt 100% đèn mạng)
      Serial.println("-> Ép sập nguồn mạch bằng chân PEN_PIN...");
      pinMode(PEN_PIN, OUTPUT); 
      digitalWrite(PEN_PIN, LOW);   
      vTaskDelay(pdMS_TO_TICKS(3000)); // Đè mức LOW đúng 3 giây để module tắt lịm
      pinMode(PEN_PIN, INPUT);
      
      Serial.println("\n=============================================");
      Serial.println("  MODULE 4G ĐÃ TẮT HOÀN TOÀN!");
      Serial.println("  (Đèn tín hiệu mạng chắc chắn đã tắt)");
      Serial.println("  -> BẠN CÓ THỂ RÚT ĐIỆN AN TOÀN NGAY BÂY GIỜ.");
      Serial.println("=============================================\n");
      
      // Đóng băng toàn bộ hệ thống ESP32 chờ rút dây điện
      while(true) { vTaskDelay(portMAX_DELAY); }
    }


    unsigned long current_time = millis();
    bool should_poll_gps = false;

    if (!has_first_fix && (current_time - last_gps_poll >= POLL_INTERVAL_NO_FIX)) {
      should_poll_gps = true;
    } else if (has_first_fix && (current_time - last_gps_poll >= POLL_INTERVAL_FIXED)) {
      should_poll_gps = true;
    }

    if (should_poll_gps) {
      last_gps_poll = current_time;
      String resGPS = sendAT("AT+QGPSLOC=2", 1500);

      if (resGPS.indexOf("+QGPSLOC:") != -1) {
        int locIndex = resGPS.indexOf("+QGPSLOC:");
        int firstComma = resGPS.indexOf(',', locIndex);         
        int secondComma = resGPS.indexOf(',', firstComma + 1);  
        int thirdComma = resGPS.indexOf(',', secondComma + 1);  
        last_lat = resGPS.substring(firstComma + 1, secondComma);
        last_lon = resGPS.substring(secondComma + 1, thirdComma);

        if (!has_first_fix) has_first_fix = true;

        if (currentState == STATE_IDLE && !trigger_sos_mqtt) {
          String payload = "{\"lat\":" + last_lat + 
                           ", \"lng\":" + last_lon + 
                           ", \"g_force\":" + String(total_G) + 
                           ", \"pitch\":" + String(angle_pitch) + 
                           ", \"roll\":" + String(angle_roll) + 
                           ", \"status\":\"TRACKING\"}";
          publishMQTT_AT(mqtt_topic, payload);
        }
      } 
    }

    if (trigger_sos_mqtt) {
      trigger_sos_mqtt = false; 
      String t_lat = (last_lat != "") ? last_lat : "\"UNKNOWN\"";
      String t_lon = (last_lon != "") ? last_lon : "\"UNKNOWN\"";

      String payload = "{\"lat\":" + t_lat + 
                       ", \"lng\":" + t_lon + 
                       ", \"g_force\":" + String(total_G) + 
                       ", \"pitch\":" + String(angle_pitch) + 
                       ", \"roll\":" + String(angle_roll) + 
                       ", \"status\":\"CRASH_DETECTED\"}";
      
      publishMQTT_AT(mqtt_topic, payload);
    }

    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

// ================= SETUP CHÍNH =================
void setup() {
  Serial.begin(115200);
  module4G.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(CANCEL_BUTTON_PIN, INPUT_PULLUP);
  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x18); Wire.endTransmission(true);
  calibrateMPU();

  Serial.println("\n========================================");
  Serial.println("  KHỞI ĐỘNG HỆ ĐIỀU HÀNH RTOS QUẢN GIA AI");
  Serial.println("========================================");

  xTaskCreatePinnedToCore(ModemTask, "ModemTask", 10000, NULL, 1, &ModemTaskHandle, 0);
  xTaskCreatePinnedToCore(TaskCoreLogic, "SensorTask", 4096, NULL, 2, &SensorTaskHandle, 1);
}

void loop() { vTaskDelete(NULL); }

