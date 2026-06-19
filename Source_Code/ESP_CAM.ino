#include "esp_camera.h" 
#include <WiFi.h> 
#include <Wire.h> 
#include <PCF8574.h> 
#include <ESP32Servo.h> 
#define CAMERA_MODEL_AI_THINKER 
#include "camera_pins.h" 
PCF8574 pcf8574(0x20); 
Servo lockServo; 
char keyMap[4][4] = { 
{ '1', '2', '3', 'A' }, 
{ '4', '5', '6', 'B' }, 
{ '7', '8', '9', 'C' }, 
{ '*', '0', '#', 'D' } 
}; 
String enteredPIN = ""; 
String correctPIN = "8421"; 
int wrongAttempts = 0; 
unsigned long blockTime = 0; 
bool blocked = false; 
const char* ssid = "Google pixel 9"; 
const char* password = "12345678911"; 
void startCameraServer(); 
void unlockDoor() { 
Serial.println("Door Unlocked"); 
lockServo.write(90); 
delay(7000); 
lockServo.write(0); 
} 
void setup() { 
  Serial.begin(115200); 
  Serial.setDebugOutput(true); 
  Wire.begin(14, 15); 
  pcf8574.begin(); 
  lockServo.setPeriodHertz(50); 
  lockServo.attach(2, 500, 2400); 
  delay(500); 
  lockServo.write(0); 
  camera_config_t config; 
  config.ledc_channel = LEDC_CHANNEL_0; 
  config.ledc_timer = LEDC_TIMER_0; 
  config.pin_d0 = Y2_GPIO_NUM; 
  config.pin_d1 = Y3_GPIO_NUM; 
  config.pin_d2 = Y4_GPIO_NUM; 
  config.pin_d3 = Y5_GPIO_NUM; 
  config.pin_d4 = Y6_GPIO_NUM; 
  config.pin_d5 = Y7_GPIO_NUM; 
  config.pin_d6 = Y8_GPIO_NUM; 
  config.pin_d7 = Y9_GPIO_NUM; 
  config.pin_xclk = XCLK_GPIO_NUM; 
  config.pin_pclk = PCLK_GPIO_NUM; 
  config.pin_vsync = VSYNC_GPIO_NUM; 
  config.pin_href = HREF_GPIO_NUM; 
  config.pin_sscb_sda = SIOD_GPIO_NUM; 
  config.pin_sscb_scl = SIOC_GPIO_NUM; 
  config.pin_pwdn = PWDN_GPIO_NUM; 
  config.pin_reset = RESET_GPIO_NUM; 
  config.xclk_freq_hz = 20000000; 
  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size = FRAMESIZE_QVGA; 
  config.jpeg_quality = 15; 
  config.fb_count = 1; 
  esp_err_t err = esp_camera_init(&config); 
  if (err != ESP_OK) { 
    Serial.printf("Camera init failed 0x%x\n", err); 
    return; 
  } 
  sensor_t* s = esp_camera_sensor_get(); 
  s->set_vflip(s, 1); 
  s->set_hmirror(s, 1); 
  s->set_framesize(s, FRAMESIZE_QVGA); 
  WiFi.begin(ssid, password); 
  unsigned long wifiStart = millis(); 
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 
10000) { 
    delay(500); 
    Serial.print("."); 
  } 
  if (WiFi.status() == WL_CONNECTED) { 
    Serial.println("\nWiFi connected"); 
    startCameraServer(); 
    Serial.print("Camera Ready: http://"); 
    Serial.println(WiFi.localIP()); 
  } else { 
    Serial.println("\nWiFi not connected, keypad still active"); 
  }
  } 
void loop() { 
  for (int row = 0; row < 4; row++) { 
    for (int i = 0; i < 8; i++) { 
      pcf8574.write(i, HIGH); 
    } 
    pcf8574.write(row, LOW); 
    for (int col = 4; col < 8; col++) { 
      if (pcf8574.read(col) == LOW) { 
        char key = keyMap[row][col - 4]; 
        Serial.print("Pressed: "); 
        Serial.println(key); 
        if (key == '#') { 
          if (blocked) { 
            if (millis() - blockTime > 10000) { 
              blocked = false; 
              wrongAttempts = 0; 
            } else { 
              enteredPIN = ""; 
              delay(300); 
              return; 
            } 
          } 
          if (enteredPIN == correctPIN) { 
            unlockDoor(); 
            wrongAttempts = 0; 
          } else { 
            wrongAttempts++;
             if (wrongAttempts >= 4) { 
              blocked = true; 
              blockTime = millis(); 
            } 
          } 
          enteredPIN = ""; 
        } else { 
          enteredPIN += key; 
        } 
        delay(300); 
      } 
    } 
  } 
} 
