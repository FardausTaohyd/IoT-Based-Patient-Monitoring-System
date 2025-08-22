#include <MAX30100.h> // Heart rate & SpO2 sensor

#include <MAX30100.h> // (Duplicate, can be removed)

/*
  ESP32 Patient Monitor -> ThingSpeak + 1602 I2C LCD
  ThingSpeak fields (order):
    1: Temperature (°C)
    2: Humidity (%)
    3: Heart Rate (BPM)
    4: SpO2 (%)
    5: gAcc (accel magnitude, g)
    6: PIR (0/1)
    7: Vibration (0/1)
    8: FallFlag (0/1)
*/

#include <WiFi.h>                // WiFi connectivity
#include <HTTPClient.h>          // HTTP requests
#include <Wire.h>                // I2C communication
#include <Adafruit_MPU6050.h>    // Accelerometer/Gyro
#include <Adafruit_Sensor.h>     // Sensor base class
#include "DHT.h"                 // DHT sensor (Temp/Humidity)
#include <ESP32Servo.h>          // Servo control
#include <MAX30100.h>            // Heart rate & SpO2 sensor
#include <LiquidCrystal_I2C.h>   // I2C LCD

// ===== USER CONFIG =====
#define WIFI_SSID          "iQOO"           // WiFi SSID
#define WIFI_PASS          "12345678"       // WiFi password
#define THINGSPEAK_API_KEY "M5ZM51Y1Y1EE6ATT" // ThingSpeak API key
// =======================

// Pin definitions
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_DHT     14
#define PIN_PIR     27
#define PIN_SW420   26          // SW-420 vibration sensor digital pin
// #define PIN_SW420_AO 34      // Optional analog input
#define PIN_BUZZER  25
#define PIN_SERVO   18
#define PIN_MPU_INT 5
#define PIN_MAX_INT 17

#define DHTTYPE DHT22          // DHT sensor type

// Global objects for sensors and actuators
Adafruit_MPU6050 mpu;
DHT dht(PIN_DHT, DHTTYPE);
MAX30100 max30100;
Servo doorServo;
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD at I2C address 0x27, 16x2

// Timing variables for periodic tasks
unsigned long lastFastRead = 0;
unsigned long lastDHTms    = 0;
unsigned long lastTS       = 0;
unsigned long lastWifiChk  = 0;
unsigned long lastLcdFlip  = 0;

// Timing intervals (ms)
const unsigned long FAST_INTERVAL_MS = 10;     // PPG + motion
const unsigned long DHT_INTERVAL_MS  = 3000;   // 3s
const unsigned long TS_INTERVAL_MS   = 18000;  // >=16s for ThingSpeak free tier
const unsigned long WIFI_CHECK_MS    = 5000;
const unsigned long LCD_FLIP_MS      = 2000;   // LCD line 2 flip every 2s

// Sensor data variables
float tempC = NAN, hum = NAN;
float ax=0, ay=0, az=0;
float gAcc = NAN;
int pirState = 0, vibState = 0;

// SW-420 vibration sensor debounce variables
int  vibLastRaw = 0;
int  vibStable  = 0;
unsigned long vibLastChange = 0;
const unsigned long VIB_DEBOUNCE_MS = 50;

// Fall detection variables
volatile bool mpuIntFlag = false;
bool fallDetected = false;
String fallSide = "NONE";
bool spikeOccurred = false;
unsigned long spikeTime = 0;

const float    FALL_SPIKE_G     = 2.5;    // Acceleration threshold for fall
const uint32_t FALL_INACT_MS    = 1200;   // Inactivity time after spike
const float    LOW_MOTION_G     = 1.12;   // Low motion threshold
const float    LEFT_RIGHT_TILT_G= 0.30;   // Tilt threshold for left/right

// Vitals (MAX30100 gives HR & SpO2)
float hrBPM = NAN, spo2 = NAN;

// PPG estimator variables (non-clinical)
const float PPG_DC_ALPHA  = 0.995f;
const float PPG_AC_ALPHA  = 0.90f;
float irDC = 0, redDC = 0;
float irAC = 0, redAC = 0;
bool  lastAbove = false;
unsigned long lastBeatAt = 0;
float bpmEMA = NAN;
unsigned long lastSpo2Compute = 0;

// Triggers
const int   HR_CRIT_LOW = 1;    // BPM near zero threshold
bool openDoorPending = false;

// Interrupt handler for MPU6050
void IRAM_ATTR onMpuInt() { mpuIntFlag = true; }

// Buzzer beep function
void beep(uint16_t ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

// Alert function: buzzer + set door open flag
void alertBuzzerDoor(const char* reason) {
  Serial.print("[ALERT] "); Serial.println(reason);
  beep(300); delay(120); beep(300);
  openDoorPending = true;
}

// Ensure WiFi is connected, try to reconnect if not
void wifiEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("[WiFi] Reconnecting...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(200); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) { Serial.print("[WiFi] OK: "); Serial.println(WiFi.localIP()); }
  else { Serial.println("[WiFi] Failed"); }
}

// Send data to ThingSpeak
bool sendToThingSpeak(float t, float h, float hr, float ox, float g, int pir, int vib, int fallFlag, const String& fallSideStr) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  String url = "http://api.thingspeak.com/update";
  String payload = "api_key=" + String(THINGSPEAK_API_KEY);
  payload += "&field1=" + String(isnan(t)?0:t,1);
  payload += "&field2=" + String(isnan(h)?0:h,1);
  payload += "&field3=" + String(isnan(hr)?0:hr,0);
  payload += "&field4=" + String(isnan(ox)?0:ox,0);
  payload += "&field5=" + String(isnan(g)?0:g,2);
  payload += "&field6=" + String(pir);
  payload += "&field7=" + String(vib);
  payload += "&field8=" + String(fallFlag);
  payload += "&status=FallSide:" + fallSideStr;

  http.begin(url);
  http.setUserAgent("ESP32-ThingSpeak/1.0");
  http.setConnectTimeout(6000);
  http.setTimeout(8000);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST(payload);
  String resp = http.getString();
  http.end();

  Serial.printf("[TS] HTTP %d Resp: %s\n", code, resp.c_str());
  return (code == 200 && resp.toInt() > 0);
}

// LCD: show temperature/humidity on line 1; flip line 2 between SpO2 and BPM
void lcdShowAll(float t, float h, float o2, float bpm) {
  // Line 1: Temperature and Humidity
  lcd.setCursor(0,0);
  char l1[17];
  // Format: "T:25.3C H:55%" fits 16 chars
  if (isnan(t) && isnan(h)) snprintf(l1, sizeof(l1), "T:--.-C H:--%%");
  else if (isnan(t))        snprintf(l1, sizeof(l1), "T:--.-C H:%2.0f%%", h);
  else if (isnan(h))        snprintf(l1, sizeof(l1), "T:%4.1fC H:--%%", t);
  else                      snprintf(l1, sizeof(l1), "T:%4.1fC H:%2.0f%%", t, h);
  lcd.print(l1);
  // Ensure full line is overwritten
  int len1 = strlen(l1);
  for (int i=len1; i<16; i++) lcd.print(' ');

  // Flip line 2 every LCD_FLIP_MS
  static bool flip = false;
  if (millis() - lastLcdFlip > LCD_FLIP_MS) { flip = !flip; lastLcdFlip = millis(); }

  // Line 2: SpO2 and BPM, alternate order
  lcd.setCursor(0,1);
  char l2[17];
  if (!flip) {
    // "SpO2:98%  BPM:75"
    int spo = isnan(o2) ? -1 : (int)round(o2);
    int hb  = isnan(bpm) ? -1 : (int)round(bpm);
    if (spo<0 && hb<0) snprintf(l2, sizeof(l2), "SpO2:--%%  BPM:--");
    else if (spo<0)    snprintf(l2, sizeof(l2), "SpO2:--%%  BPM:%2d", hb);
    else if (hb<0)     snprintf(l2, sizeof(l2), "SpO2:%2d%%  BPM:--", spo);
    else               snprintf(l2, sizeof(l2), "SpO2:%2d%%  BPM:%2d", spo, hb);
  } else {
    // Alternate ordering "BPM:75  SpO2:98%"
    int spo = isnan(o2) ? -1 : (int)round(o2);
    int hb  = isnan(bpm) ? -1 : (int)round(bpm);
    if (hb<0 && spo<0) snprintf(l2, sizeof(l2), "BPM:--  SpO2:--%%");
    else if (hb<0)     snprintf(l2, sizeof(l2), "BPM:--  SpO2:%2d%%", spo);
    else if (spo<0)    snprintf(l2, sizeof(l2), "BPM:%2d  SpO2:--%%", hb);
    else               snprintf(l2, sizeof(l2), "BPM:%2d  SpO2:%2d%%", hb, spo);
  }
  lcd.print(l2);
  int len2 = strlen(l2);
  for (int i=len2; i<16; i++) lcd.print(' ');
}

// PPG processing: estimate HR & SpO2 (non-clinical)
void processPPG() {
  max30100.readSensor();           // Read raw IR and Red values
  uint16_t rawIR  = max30100.IR;   // IR value
  uint16_t rawRed = max30100.RED;  // Red value

  // DC and AC component estimation (exponential moving average)
  irDC  = PPG_DC_ALPHA * irDC  + (1.0f - PPG_DC_ALPHA) * rawIR;
  redDC = PPG_DC_ALPHA * redDC + (1.0f - PPG_DC_ALPHA) * rawRed;
  float irDet  = rawIR  - irDC;
  float redDet = rawRed - redDC;
  irAC  = PPG_AC_ALPHA * irAC  + (1.0f - PPG_AC_ALPHA) * fabs(irDet);
  redAC = PPG_AC_ALPHA * redAC + (1.0f - PPG_AC_ALPHA) * fabs(redDet);

  // Heartbeat detection (rising edge)
  bool above = irDet > (0.6f * irAC);
  unsigned long now = millis();
  if (above && !lastAbove) {
    if (lastBeatAt != 0) {
      unsigned long ibi = now - lastBeatAt;
      if (ibi > 300 && ibi < 2000) {
        float bpm = 60000.0f / ibi;
        if (isnan(bpmEMA)) bpmEMA = bpm;
        bpmEMA = 0.85f * bpmEMA + 0.15f * bpm;
        hrBPM = bpmEMA;
      }
    }
    lastBeatAt = now;
  }
  lastAbove = above;

  // SpO2 estimation (once per second)
  if (now - lastSpo2Compute > 1000) {
    lastSpo2Compute = now;
    if (irAC > 1 && redAC > 1 && irDC > 100 && redDC > 100) {
      float R = (redAC / redDC) / (irAC / irDC);
      float est = 110.0f - 25.0f * R; // rough, non-clinical
      if (est > 100) est = 100;
      if (est < 70)  est = 70;
      spo2 = est;
    }
  }
}

// MPU6050 setup
bool setupMPU() {
  if (!mpu.begin()) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  pinMode(PIN_MPU_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), onMpuInt, RISING);
  return true;
}

// MAX30100 setup
bool setupMAX30100() {
  max30100.begin();

  // Preferred config calls (uncomment if needed)
  // max30100.setMode(MAX30100_MODE_SPO2);
  // max30100.setLedsCurrent(MAX30100_LED_CURR_24_0MA, MAX30100_LED_CURR_24_0MA);
  // max30100.setSamplingRate(MAX30100_SAMPRATE_100HZ);
  // max30100.setLedsPulseWidth(MAX30100_SPC_PW_1600US);

  irDC = redDC = 0; irAC = redAC = 0;
  lastAbove = false; lastBeatAt = 0; bpmEMA = NAN; lastSpo2Compute = 0;
  return true;
}

// Arduino setup function
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_SW420, INPUT);
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);

  dht.begin();
  lcd.init(); lcd.backlight();
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Patient Monitor");
  lcd.setCursor(0,1); lcd.print("Booting...");

  if (!setupMPU()) Serial.println("[Init] MPU6050 init failed");
  if (!setupMAX30100()) Serial.println("[Init] MAX30100 init failed");

  // Servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  doorServo.setPeriodHertz(50);
  doorServo.attach(PIN_SERVO, 500, 2500);
  doorServo.write(0);

  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) { delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) { Serial.print("[WiFi] "); Serial.println(WiFi.localIP()); }
  else { Serial.println("[WiFi] connect timeout"); }

  lcd.clear();
}

// Arduino main loop
void loop() {
  unsigned long now = millis();

  // Periodically check WiFi connection
  if (now - lastWifiChk >= WIFI_CHECK_MS) { lastWifiChk = now; wifiEnsureConnected(); }

  // Fast periodic tasks: PPG, motion, PIR, vibration
  if (now - lastFastRead >= FAST_INTERVAL_MS) {
    lastFastRead = now;

    processPPG(); // Heart rate & SpO2

    // Read accelerometer/gyro
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      gAcc = sqrt(ax*ax + ay*ay + az*az) / 9.80665;

      // Fall detection logic
      if (gAcc > FALL_SPIKE_G) { spikeOccurred = true; spikeTime = now; }
      if (spikeOccurred && (now - spikeTime > FALL_INACT_MS)) {
        if (gAcc < LOW_MOTION_G) {
          if (ay >  LEFT_RIGHT_TILT_G) fallSide = "RIGHT";
          else if (ay < -LEFT_RIGHT_TILT_G) fallSide = "LEFT";
          else fallSide = "UNSURE";
          fallDetected = true;
        }
        spikeOccurred = false;
      }
    }

    pirState = digitalRead(PIN_PIR); // PIR sensor

    // SW-420 vibration sensor debounce
    int vibRaw = digitalRead(PIN_SW420);
    if (vibRaw != vibLastRaw) {
      vibLastRaw = vibRaw;
      vibLastChange = now;
    }
    if (now - vibLastChange > VIB_DEBOUNCE_MS) {
      vibStable = vibRaw;
    }
    vibState = vibStable;

    // Optional analog mode:
    // int vibAnalog = analogRead(PIN_SW420_AO);
    // vibState = (vibAnalog > VIB_ANALOG_THRESHOLD) ? 1 : 0;
  }

  // Read DHT sensor and update LCD
  if (now - lastDHTms >= DHT_INTERVAL_MS) {
    lastDHTms = now;
    tempC = dht.readTemperature();
    hum   = dht.readHumidity();
    lcdShowAll(tempC, hum, spo2, hrBPM);
  }

  // Critical BPM or vibration triggers alert
  bool bpmCritical = (isnan(hrBPM) || hrBPM <= HR_CRIT_LOW);
  if (bpmCritical) alertBuzzerDoor("BPM near zero");
  if (vibState == 1) alertBuzzerDoor("Vibration detected");

  // Open door if alert triggered
  if (openDoorPending) {
    doorServo.write(90);
    delay(2000);
    doorServo.write(0);
    openDoorPending = false;
  }

  // Periodically send data to ThingSpeak
  if (now - lastTS >= TS_INTERVAL_MS) {
    lastTS = now;
    bool ok = sendToThingSpeak(tempC, hum, hrBPM, spo2, gAcc, pirState, vibState, fallDetected ? 1 : 0, fallSide);
    if (!ok) { delay(2000); sendToThingSpeak(tempC, hum, hrBPM, spo2, gAcc, pirState, vibState, fallDetected ? 1 : 0, fallSide); }
    fallDetected = false;
  }
}
