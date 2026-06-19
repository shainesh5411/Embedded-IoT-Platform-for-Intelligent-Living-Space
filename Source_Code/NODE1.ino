#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#define REMOTEXY_MODE__WIFI_CLOUD
#include <WiFi.h>
// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "Living_Space"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "b64bd1a75922e9b5bc813fd3088f719b"
#include <RemoteXY.h>
// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =  // 287 bytes V19
  { 255, 3, 0, 12, 0, 24, 1, 19, 0, 0, 0, 78, 79, 68, 69, 49, 0, 31, 1, 106,
    200, 1, 1, 17, 0, 73, 7, 15, 26, 40, 12, 128, 0, 177, 26, 0, 0, 0, 0, 0,
    0, 200, 66, 0, 0, 0, 0, 67, 10, 59, 21, 9, 77, 2, 26, 2, 2, 9, 72, 22,
    10, 1, 2, 26, 135, 37, 79, 78, 0, 79, 70, 70, 0, 67, 71, 59, 20, 9, 77, 2,
    26, 2, 2, 70, 72, 22, 10, 1, 2, 26, 135, 37, 79, 78, 0, 79, 70, 70, 0, 129,
    3, 88, 36, 7, 64, 177, 87, 97, 116, 101, 114, 32, 84, 97, 110, 107, 0, 129, 60, 88,
    41, 7, 64, 134, 83, 111, 105, 108, 32, 77, 111, 105, 115, 116, 117, 114, 101, 0, 2, 14,
    135, 23, 9, 1, 2, 26, 135, 37, 79, 78, 0, 79, 70, 70, 0, 129, 4, 151, 46, 8,
    64, 132, 71, 97, 114, 100, 101, 110, 32, 76, 105, 103, 104, 116, 0, 72, 59, 18, 44, 44,
    12, 166, 140, 120, 26, 0, 0, 0, 0, 0, 0, 200, 66, 0, 0, 0, 0, 129, 30, 172,
    42, 12, 64, 9, 78, 79, 68, 69, 32, 49, 0, 129, 13, 4, 14, 10, 64, 17, 240, 159,
    146, 167, 0, 129, 72, 3, 17, 12, 64, 17, 240, 159, 140, 179, 0, 129, 17, 117, 17, 12,
    64, 17, 240, 159, 146, 161, 0, 65, 74, 132, 14, 14, 97, 129, 60, 150, 41, 8, 64, 205,
    82, 97, 105, 110, 32, 83, 116, 97, 116, 117, 115, 0, 129, 73, 117, 16, 11, 64, 17, 240,
    159, 140, 167, 239, 184, 143, 0 };

// this structure defines all the variables and events of your control interface
struct {

  // input variables
  uint8_t t_sw;           // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t s_sw;           // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t grdn_light_sw;  // =1 if switch ON and =0 if OFF, from 0 to 1

  // output variables
  int8_t t_level;  // from 0 to 100
  float tnk_level_value;
  float soil_level_value;
  int8_t s_level;   // from 0 to 100
  uint8_t rain;     // =0..255 LED Red brightness, from 0 to 255
  uint8_t no_rain;  // =0..255 LED Green brightness, from 0 to 255

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
/* ---------- PINS ---------- */
#define TANK_PUMP 25
#define GARDEN_PUMP 26
#define GARDEN_LIGHT 33
#define MOISTURE_SENSOR 34

#define TRIGPIN 27
#define ECHOPIN 14

/* ---------- OLED ---------- */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define RAIN_SENSOR_PIN 5  // Rain sensor digital pin
#define SERVO_PIN 18       // Servo signal pin

Servo rainServo;

/* ---------- RTC ---------- */
RTC_DS3231 rtc;

/* ---------- Tank Calibration ---------- */
int fullTankDistance = 2;    // tank full
int emptyTankDistance = 10;  // tank empty

/* ---------- Soil Calibration ---------- */
int wetSoilVal = 1800;
int drySoilVal = 3800;

/* ---------- Variables ---------- */
long duration;
float distance;

int waterLevelPer;
int moisturePercent;
int sensorVal;

/* ---------- Demo Pump Variables ---------- */
bool demoPump = false;
int lastTriggeredMinute = -1;
unsigned long pumpStartTime = 0;

bool gardenLight = false;
bool watertnk = false;
bool gardenpump = false;
bool rain_yes = false;
bool rain_no = false;

/* ---------- Setup ---------- */
void setup() {

  Serial.begin(115200);

  pinMode(TANK_PUMP, OUTPUT);
  pinMode(GARDEN_PUMP, OUTPUT);
  pinMode(GARDEN_LIGHT, OUTPUT);
  digitalWrite(TANK_PUMP, LOW);
  digitalWrite(GARDEN_PUMP, LOW);
  digitalWrite(GARDEN_LIGHT, LOW);


  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(RAIN_SENSOR_PIN, INPUT);

  rainServo.attach(SERVO_PIN);  // attach servo to GPIO18

  rainServo.write(0);  // initial position (OPEN)
  analogReadResolution(12);

  Wire.begin();

  rtc.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED FAIL");
    while (1)
      ;
  }

  display.clearDisplay();

  RemoteXY_Init();  // initialization by macros
}

void rain_measure() {

  int rainState = digitalRead(RAIN_SENSOR_PIN);

  if (rainState == LOW)  // Rain detected
  {
    rainServo.write(90);  // Close cover
    rain_yes = true;
    rain_no = false;
  }

  else {
    rainServo.write(0);  // Open cover
    rain_yes = false;
    rain_no = true;
  }
}
/* ---------- Measure Tank Distance ---------- */
void measureDistance() {

  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);

  duration = pulseIn(ECHOPIN, HIGH, 30000);

  distance = duration * 0.034 / 2;

  if (distance > emptyTankDistance)
    waterLevelPer = 0;

  else if (distance < fullTankDistance)
    waterLevelPer = 100;

  else
    waterLevelPer = map(distance, emptyTankDistance, fullTankDistance, 0, 100);

  waterLevelPer = constrain(waterLevelPer, 0, 100);
}

/* ---------- Soil Moisture ---------- */
void readMoisture() {

  sensorVal = analogRead(MOISTURE_SENSOR);

  moisturePercent = map(sensorVal, drySoilVal, wetSoilVal, 0, 100);
  moisturePercent = constrain(moisturePercent, 0, 100);
}

/* ---------- Tank Pump Logic ---------- */
void tankPumpLogic() {

  DateTime now = rtc.now();

  /* DEMO every 5 minutes */

  if ((now.minute() % 5 == 0) && now.minute() != lastTriggeredMinute) {

    demoPump = true;
    pumpStartTime = millis();
    lastTriggeredMinute = now.minute();
  }

  if (demoPump) {
    digitalWrite(TANK_PUMP, HIGH);
    watertnk = true;
    if (millis() - pumpStartTime > 10000) {
      demoPump = false;
      digitalWrite(TANK_PUMP, LOW);
      watertnk = false;
    }
    return;
  }

  /* Normal Tank Control */
  if ((RemoteXY.t_sw == 1 && waterLevelPer < 85) || (waterLevelPer < 25)) {
    digitalWrite(TANK_PUMP, HIGH);  // Manual ON
    watertnk = true;
  }

  else if (waterLevelPer > 85 || RemoteXY.t_sw == 0) {
    digitalWrite(TANK_PUMP, LOW);  // OFF
    watertnk = false;
  }

}
/* ---------- Garden Pump Logic ---------- */
void gardenPumpLogic() {

  if ((RemoteXY.s_sw == 1 && moisturePercent < 85) || (moisturePercent < 25)) {
    digitalWrite(GARDEN_PUMP, HIGH);
    gardenpump = true;
  }

  else if (((RemoteXY.t_sw == 0 && moisturePercent > 25) || moisturePercent > 85)) {
    digitalWrite(GARDEN_PUMP, LOW);
    gardenpump = false;
  }
  if (RemoteXY.grdn_light_sw == 1) {
    digitalWrite(GARDEN_LIGHT, HIGH);  // OFF
    gardenLight = true;
    // relayStateL = false;
  } else if (RemoteXY.grdn_light_sw == 0) {
    digitalWrite(GARDEN_LIGHT, LOW);  // OFF
    gardenLight = false;
    //relayStateL = false;
  }
}

/* ---------- OLED Display ---------- */
void displayData() {

  DateTime now = rtc.now();

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Time:");
  display.print(now.hour());
  display.print(":");
  display.print(now.minute());
  display.print(":");
  display.println(now.second());

  // display.setCursor(0,10);
  display.print("Water:");
  display.print(waterLevelPer);
  display.print("% ");
  display.print(" Light:");
  display.println(gardenLight ? "ON" : "OFF");

  // display.setCursor(0,20);
  display.print("Moist:");
  display.print(moisturePercent);
  display.print("% ");
  display.print(" Rain: ");
  display.print(rain_yes ? "YES" : "NO");

  display.display();
}
/* ---------- LOOP ---------- */
void loop() {

  measureDistance();
  readMoisture();

  tankPumpLogic();
  gardenPumpLogic();
  rain_measure();
  displayData();

  RemoteXYEngine.handler();
  RemoteXY.t_sw = watertnk;
  RemoteXY.s_sw = gardenpump;
  RemoteXY.t_level = waterLevelPer;
  RemoteXY.tnk_level_value = waterLevelPer;
  RemoteXY.s_level = moisturePercent;
  RemoteXY.soil_level_value = moisturePercent;
  RemoteXY.rain = (rain_yes ? 255 : 0);
  RemoteXY.no_rain = (rain_no ? 255 : 0);

  Serial.print("TANK: ");
  Serial.print(waterLevelPer);
  Serial.print("  MOISTURE: ");
  Serial.print(moisturePercent);
  Serial.print("  GARDEN_LIGHT: ");
  Serial.print(gardenLight ? "ON" : "OFF");
  Serial.print("  RAIN: ");
  Serial.println(rain_yes ? "YES" : "NO");
  delay(800);
}