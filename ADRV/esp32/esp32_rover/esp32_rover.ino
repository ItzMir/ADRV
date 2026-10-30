#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <ESP32Servo.h>
#include "secrets.h"

// ---------- WiFi ----------
const char* SSID     = WIFI_SSID;
const char* PASSWORD = WIFI_PASSWORD;
WebServer server(80);

// ---------- Pi UART ----------
HardwareSerial SerialPi(2);
unsigned long lastPiCommand = 0;

// ---------- MLX90640 ----------
Adafruit_MLX90640 mlx;
float thermalFrame[32 * 24];

// ---------- Pan servo ----------
#define PAN_SERVO_PIN 13
Servo panServo;
int panAngle = 90;

// ---------- Shaft servos ----------
//   LEFT : idle 0°  → active 90°  (sweeps outward left)
//   RIGHT: idle 90° → active 0°   (sweeps outward right)
#define SHAFT_LEFT_PIN   2
#define SHAFT_RIGHT_PIN  23
Servo shaftLeft;
Servo shaftRight;

#define SHAFT_L_IDLE    0
#define SHAFT_L_ACTIVE  90
#define SHAFT_R_IDLE    90
#define SHAFT_R_ACTIVE  0

// Trigger distance for shaft sweep — must match config.py STOP_DISTANCE_CM
const int OBSTACLE_TRIGGER_CM = 10;

enum ShaftState { SHAFT_IDLE, SHAFT_ACTIVE, SHAFT_COOLDOWN };
ShaftState shaftState = SHAFT_IDLE;
unsigned long shaftStateStart = 0;
const unsigned long SHAFT_ACTIVE_MS   = 900;
const unsigned long SHAFT_COOLDOWN_MS = 600;

bool humanNear = false;
unsigned long lastHumanMsg = 0;
const unsigned long HUMAN_TIMEOUT = 2000;

// ---------- Ultrasonic ----------
#define TRIG_FRONT 4
#define ECHO_FRONT 18
#define TRIG_LEFT  5
#define ECHO_LEFT  19
#define TRIG_RIGHT 32
#define ECHO_RIGHT 33
#define MAX_DISTANCE 200

int lastFrontCm = 200;

// ---------- Utilities ----------
int readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return MAX_DISTANCE;
  int d = duration * 0.0343 / 2;
  if (d > MAX_DISTANCE) d = MAX_DISTANCE;
  if (d < 1) d = 1;
  return d;
}

int getFrontDist() { return readDistance(TRIG_FRONT, ECHO_FRONT); }
int getLeftDist()  { return readDistance(TRIG_LEFT,  ECHO_LEFT);  }
int getRightDist() { return readDistance(TRIG_RIGHT, ECHO_RIGHT); }

void readAllSensors(int &f, int &l, int &r) {
  f = getFrontDist(); delay(60);
  l = getLeftDist();  delay(60);
  r = getRightDist(); delay(60);
}

void setPan(int angle) {
  if (angle < 15)  angle = 15;
  if (angle > 165) angle = 165;
  panServo.write(angle);
  panAngle = angle;
}

void shaftLeftIdle()   { shaftLeft.write(SHAFT_L_IDLE);   }
void shaftLeftActive() { shaftLeft.write(SHAFT_L_ACTIVE); }
void shaftRightIdle()   { shaftRight.write(SHAFT_R_IDLE);   }
void shaftRightActive() { shaftRight.write(SHAFT_R_ACTIVE); }

void updateShafts() {
  switch (shaftState) {
    case SHAFT_IDLE:
      if (lastFrontCm <= OBSTACLE_TRIGGER_CM && !humanNear) {
        shaftLeftActive();
        shaftRightActive();
        shaftState = SHAFT_ACTIVE;
        shaftStateStart = millis();
      }
      break;
    case SHAFT_ACTIVE:
      if (millis() - shaftStateStart >= SHAFT_ACTIVE_MS) {
        shaftLeftIdle();
        shaftRightIdle();
        shaftState = SHAFT_COOLDOWN;
        shaftStateStart = millis();
      }
      break;
    case SHAFT_COOLDOWN:
      if (millis() - shaftStateStart >= SHAFT_COOLDOWN_MS) {
        shaftState = SHAFT_IDLE;
      }
      break;
  }
}

void analyzeThermal(float &maxT, int &maxR, int &maxC,
                    float &centR, float &centC, float &minT) {
  maxT = -1000.0f; minT = 1000.0f;
  maxR = maxC = 0;

  for (int r = 0; r < 24; r++) {
    for (int c = 0; c < 32; c++) {
      float t = thermalFrame[r * 32 + c];
      if (t > maxT) { maxT = t; maxR = r; maxC = c; }
      if (t < minT) minT = t;
    }
  }
  float sumR = 0, sumC = 0; int n = 0;
  float thr = maxT - 2.0f;
  for (int r = 0; r < 24; r++) {
    for (int c = 0; c < 32; c++) {
      if (thermalFrame[r * 32 + c] >= thr) { sumR += r; sumC += c; n++; }
    }
  }
  if (n > 0) { centR = sumR / n; centC = sumC / n; }
  else       { centR = maxR;     centC = maxC;     }
}

void sendTelemetry() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  last = millis();

  int f, l, r;
  readAllSensors(f, l, r);
  lastFrontCm = f;

  StaticJsonDocument<256> doc;
  doc["front_cm"]    = f;
  doc["left_cm"]     = l;
  doc["right_cm"]    = r;
  doc["obstacle"]    = (f < 30);
  doc["pan"]         = panAngle;
  doc["human_near"]  = humanNear;
  doc["shaft_state"] = (shaftState == SHAFT_IDLE)   ? "IDLE"
                     : (shaftState == SHAFT_ACTIVE) ? "ACTIVE"
                                                    : "COOLDOWN";
  String out;
  serializeJson(doc, out);
  SerialPi.println(out);
}

void sendThermal() {
  static unsigned long last = 0;
  if (millis() - last < 500) return;
  last = millis();

  if (mlx.getFrame(thermalFrame) != 0) return;

  float maxT, minT, centR, centC;
  int maxR, maxC;
  analyzeThermal(maxT, maxR, maxC, centR, centC, minT);

  StaticJsonDocument<256> doc;
  JsonObject t = doc.createNestedObject("thermal");
  t["max"]        = maxT;
  t["min"]        = minT;
  t["max_r"]      = maxR;
  t["max_c"]      = maxC;
  t["centroid_r"] = centR;
  t["centroid_c"] = centC;
  doc["pan"] = panAngle;

  String out;
  serializeJson(doc, out);
  SerialPi.println(out);
}

void handlePiCommands() {
  static String lineBuf = "";
  while (SerialPi.available()) {
    char c = SerialPi.read();
    if (c == '\n') {
      lineBuf.trim();
      if (lineBuf.length() > 0) {
        StaticJsonDocument<256> doc;
        if (!deserializeJson(doc, lineBuf)) {
          const char* cmd = doc["cmd"];
          if (cmd) {
            lastPiCommand = millis();
            if (strcmp(cmd, "PAN") == 0) {
              int a = doc["angle"] | 90;
              setPan(a);
            }
            else if (strcmp(cmd, "HUMAN_NEAR") == 0) {
              humanNear = doc["value"] | false;
              lastHumanMsg = millis();
            }
          }
        }
      }
      lineBuf = "";
    } else if (c != '\r') {
      lineBuf += c;
      if (lineBuf.length() > 200) lineBuf = "";
    }
  }
}

void handleRoot() {
  String page = "<!DOCTYPE html><html><body style='background:#111;color:#eee;"
    "font-family:sans-serif;text-align:center;padding:40px;'>"
    "<h1>ESP32 Rover Sensors</h1>"
    "<p>Use the Pi dashboard at http://PI_IP:5000</p>"
    "<p>Pan: <b>" + String(panAngle) + "</b></p>"
    "<p>Shaft: <b>" + String(shaftState == SHAFT_IDLE ? "IDLE"
                            : shaftState == SHAFT_ACTIVE ? "ACTIVE" : "COOLDOWN")
    + "</b></p></body></html>";
  server.send(200, "text/html", page);
}

void handleStatus() {
  int f, l, r;
  readAllSensors(f, l, r);
  StaticJsonDocument<256> doc;
  doc["front"] = f; doc["left"] = l; doc["right"] = r;
  doc["pan"]   = panAngle;
  doc["human_near"]  = humanNear;
  doc["shaft_state"] = (shaftState == SHAFT_IDLE)   ? "IDLE"
                     : (shaftState == SHAFT_ACTIVE) ? "ACTIVE"
                                                    : "COOLDOWN";
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handlePan() {
  if (!server.hasArg("angle")) { server.send(400, "text/plain", "no angle"); return; }
  setPan(server.arg("angle").toInt());
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  panServo.attach(PAN_SERVO_PIN);
  setPan(90);

  shaftLeft.attach(SHAFT_LEFT_PIN);
  shaftRight.attach(SHAFT_RIGHT_PIN);
  shaftLeft.write(SHAFT_L_IDLE);
  shaftRight.write(SHAFT_R_IDLE);

  SerialPi.begin(115200, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!mlx.begin(0x33, &Wire)) {
    Serial.println("MLX90640 init failed");
  } else {
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_4_HZ);
    Serial.println("MLX90640 ready");
  }

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Non-blocking WiFi: the UART link to the Pi must come up immediately,
  // even if the router is absent. The backup web page works once connected.
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/pan", handlePan);
  server.begin();
}

void loop() {
  static bool ipReported = false;
  if (!ipReported && WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
    ipReported = true;
  }
  server.handleClient();
  handlePiCommands();
  sendTelemetry();
  sendThermal();

  if (millis() - lastHumanMsg > HUMAN_TIMEOUT) {
    humanNear = false;
  }

  updateShafts();
}
