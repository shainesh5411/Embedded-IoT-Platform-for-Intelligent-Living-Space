#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// ================= PIN CONFIG =================
#define DHTPIN 4
#define DHTTYPE DHT22
#define MQ2_PIN 34
#define SERVO_PIN 18
#define EXHAUST_PIN 25
#define PIR_PIN 27
#define LDR_PIN 33
#define LED_PIN 16
// ================= OLED CONFIG =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// ================= THRESHOLDS =================
#define TEMP_THRESHOLD 38.0
#define HUM_THRESHOLD 70.0
#define GAS_THRESHOLD 2000
#define LDR_THRESHOLD 2000  // Dark = LOW value
// ================= OBJECTS =================
DHT dht(DHTPIN, DHTTYPE);
Servo windowServo;
// ================= GLOBALS =================
bool servoOpen = false;
bool exhaust = false;
bool ledOn = false;
float temperature = 0;
float humidity = 0;
//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////
// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG
// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__WIFI_CLOUD
#include <WiFi.h>
#include <EEPROM.h>
// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "Living_Space"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "74ae4f4e89c194aec0fa247258005d31"
#include <RemoteXY.h>
// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =  // 382 bytes V21
  { 254, 3, 0, 14, 0, 1, 0, 8, 0, 0, 0, 113, 1, 21, 0, 0, 0, 78, 111, 100,
    101, 50, 0, 31, 1, 106, 200, 1, 1, 28, 0, 129, 31, 182, 42, 12, 64, 9, 78, 79,
    68, 69, 32, 50, 0, 129, 3, 10, 51, 9, 64, 48, 84, 101, 109, 112, 101, 114, 97, 116,
    117, 114, 101, 0, 129, 62, 10, 36, 9, 64, 178, 72, 117, 109, 105, 100, 105, 116, 121, 0,
    67, 11, 28, 26, 13, 77, 48, 31, 2, 129, 35, 30, 11, 10, 64, 48, 194, 176, 67, 0,
    67, 63, 27, 25, 13, 77, 177, 31, 2, 129, 88, 28, 9, 11, 64, 177, 37, 0, 129, 18,
    45, 21, 12, 64, 37, 71, 97, 115, 0, 67, 15, 69, 27, 12, 85, 50, 31, 129, 69, 45,
    20, 11, 64, 10, 76, 68, 82, 0, 67, 67, 69, 26, 12, 85, 10, 31, 194, 97, 1, 8,
    8, 4, 1, 26, 31, 65, 25, 162, 13, 13, 97, 129, 13, 135, 38, 12, 64, 118, 77, 111,
    116, 105, 111, 110, 0, 129, 21, 20, 13, 9, 64, 17, 240, 159, 140, 161, 239, 184, 143, 0,
    129, 74, 20, 11, 8, 64, 17, 240, 159, 146, 167, 0, 129, 22, 58, 14, 10, 64, 17, 226,
    155, 189, 0, 129, 73, 57, 14, 10, 64, 17, 226, 152, 128, 239, 184, 143, 0, 129, 23, 148,
    17, 12, 64, 17, 240, 159, 143, 131, 0, 129, 16, 86, 27, 12, 64, 246, 76, 105, 103, 104,
    116, 0, 129, 22, 102, 16, 11, 64, 17, 240, 159, 146, 161, 0, 2, 18, 117, 23, 11, 0,
    78, 8, 134, 1, 79, 78, 0, 79, 70, 70, 0, 129, 59, 86, 40, 11, 64, 8, 69, 120,
    104, 97, 117, 115, 116, 0, 129, 72, 101, 16, 11, 64, 17, 240, 159, 146, 168, 0, 2, 68,
    117, 23, 11, 0, 78, 8, 134, 1, 79, 78, 0, 79, 70, 70, 0, 129, 64, 136, 36, 10,
    64, 164, 87, 105, 110, 100, 111, 119, 0, 129, 74, 148, 14, 10, 64, 17, 240, 159, 170, 159,
    0, 2, 58, 163, 42, 11, 0, 78, 8, 134, 1, 79, 80, 69, 78, 0, 67, 76, 79, 83,
    69, 0 };

// this structure defines all the variables and events of your control interface
struct {
  // input variables
  uint8_t light_sw;    // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t exhaust_sw;  // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t window_sw;   // =1 if switch ON and =0 if OFF, from 0 to 1
  // output variables
  float Temp_value;
  float hum_value;
  int16_t gas_value;   // -32768 .. +32767
  int16_t ldr_value;   // -32768 .. +32767
  uint8_t motion_no;   // =0..255 LED Red brightness, from 0 to 255
  uint8_t motion_yes;  // =0..255 LED Green brightness, from 0 to 255
  // complex variables
  RemoteXYType_NotificationNet Alert;  // call .print() or .println(), then .send()
} RemoteXY;
#pragma pack(pop)
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
// ================= STARTUP =================
void showStartup() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 20);
  display.println("KITCHEN");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(20, 15);
  display.println("INITIALIZING");
  display.drawRect(14, 35, 100, 10, SSD1306_WHITE);
  for (int i = 0; i <= 10; i++) {
    int fill = (98 * i) / 10;
    display.fillRect(15, 36, fill, 8, SSD1306_WHITE);
    display.display();
    delay(100);
  }
  delay(500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(25, 20);
  display.println("READY");
  display.display();
  delay(1000);
}
void setup() {
  Serial.begin(115200);
  delay(300);
  dht.begin();
  pinMode(EXHAUST_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_PIN, ADC_11db);
  digitalWrite(EXHAUST_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED INIT FAILED");
    while (true)
      ;
  }
  shwStartup();
  // Attach servo AFTER startup
  windowServo.attach(SERVO_PIN, 500, 2400);
  windowServo.write(0);
  RemoteXY_Init();  // initialization by macros
  EEPROM.begin(RemoteXYEngine.getEepromSize());  // init EEPROM
}
void loop() {
  // ===== SENSOR READ =====
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperature = t;
  if (!isnan(h)) humidity = h;
  int gasLevel = analogRead(MQ2_PIN);
  int pirValue = digitalRead(PIR_PIN);
  int ldrRaw = analogRead(LDR_PIN);
  // ===== SAFETY LOGIC =====
  bool dangerState = (temperature > TEMP_THRESHOLD || humidity > HUM_THRESHOLD || gasLevel > GAS_THRESHOLD);
  if (RemoteXY.exhaust_sw == 1)  // Manual ON
  {
    exhaust = true;
  } else if (dangerState)  // Auto ON
  {
    exhaust = true;
    RemoteXY.Alert.print("DANGER!!!");
    RemoteXY.Alert.send();
  } else  // Safe
  {
    exhaust = false;
  }
  digitalWrite(EXHAUST_PIN, exhaust ? HIGH : LOW);
  if (RemoteXY.window_sw == 1)  // Manual OPEN
  {
    servoOpen = true;
  } else if (dangerState)  // Auto OPEN
  {
    servoOpen = true;
  } else  // Safe → CLOSE
  {
    servoOpen = false;
  }
  windowServo.write(servoOpen ? 90 : 0);
  bool isDark = (ldrRaw < LDR_THRESHOLD);
  bool on_led = (pirValue == 1 && isDark);
  bool off_led = (pirValue == 0 && isDark);
  if (on_led)  // Auto mode
  {
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
  } else if (off_led)  // Auto mode
  {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  } else if (RemoteXY.light_sw == 1)  // Auto mode
  {
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
  } else if (RemoteXY.light_sw == 0)  // Manual ON
  {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  } else {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
  // ===== SERIAL OUTPUT =====
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print("  Hum: ");
  Serial.print(humidity);
  Serial.print("  Gas: ");
  Serial.print(gasLevel);
  Serial.print("  LDR: ");
  Serial.print(ldrRaw);
  Serial.print("  Motion: ");
  Serial.print(pirValue);
  Serial.print("  LED: ");
  Serial.print(ledOn ? "ON" : "OFF");
  Serial.print("  Exhaust: ");
  Serial.print(exhaust ? "ON" : "OFF");
  Serial.print("  Window: ");
  Serial.println(servoOpen ? "OPEN" : "CLOSE");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Temp : ");
  display.print((int)temperature);
  display.println(" C");
  display.print("Hum : ");
  display.print((int)humidity);
  display.println(" %");
  display.print("Gas : ");
  display.println(gasLevel);
  display.print("LDR : ");
  display.println(ldrRaw);
  display.print("Motion:");
  display.println(pirValue ? "YES" : "NO");
  display.print("Light:");
  display.println(ledOn ? "ON" : "OFF");
  display.print("Exhaust: ");
  display.println(exhaust ? "ON" : "OFF");
  display.print("Window: ");
  display.println(servoOpen ? "OPEN" : "CLOSE");
  display.display();
  RemoteXYEngine.handler();
  RemoteXY.light_sw = ledOn;
  RemoteXY.exhaust_sw = exhaust;
  RemoteXY.window_sw = servoOpen;
  RemoteXY.Temp_value = temperature;
  RemoteXY.hum_value = humidity;
  RemoteXY.gas_value = gasLevel;
  RemoteXY.ldr_value = ldrRaw;
  RemoteXY.motion_no = (pirValue ? 0 : 255);
  RemoteXY.motion_yes = (pirValue ? 255 : 0);
  delay(800);
  RemoteXYEngine.delay(2000);
}