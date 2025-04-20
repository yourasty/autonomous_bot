#include <Arduino.h>
#include <WiFiS3.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <WebSocketsServer.h>
#include "hal_data.h"
#include <EEPROM.h>
#include <Arduino_JSON.h>
#include "client.h"


#define EEPROM_ADDR_MODE 0
#define LOG_AUTO_FORWARD       "AUTO_FORWARD"
#define LOG_AUTO_REVERSE_LEFT  "AUTO_REVERSE_LEFT"
#define LOG_AUTO_REVERSE_RIGHT "AUTO_REVERSE_RIGHT"
#define LOG_AUTO_AVOID_LEFT    "AUTO_AVOID_LEFT"
#define LOG_AUTO_AVOID_RIGHT   "AUTO_AVOID_RIGHT"
#define LOG_SENSOR_FAIL        "SENSOR_FAIL"
#define LOG_MODE_SWITCH        "MODE_SWITCH"

String logs = "";

const char ssid[] = "RC_Controller";
const char pass[] = "";
WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

Servo esc, steer, sensorServo;
VL53L0X centerSensor, leftSensor, rightSensor;
const int escPin = 9;
const int steerPin = 5;
const int sensorServoPin = 6;
const int centerSensorResetPin = 3;
const int leftSensorResetPin = 1;
const int rightSensorResetPin = 2;

const int STEER_STRAIGHT = 80;
const int MAX_STEER_ANGLE_LEFT = 50;
const int MAX_STEER_ANGLE_RIGHT = 40;
const int STEER_RIGHT_MAX = STEER_STRAIGHT + MAX_STEER_ANGLE_RIGHT;
const int STEER_LEFT_MAX = STEER_STRAIGHT - MAX_STEER_ANGLE_LEFT;
const int NEUTRAL = 1500;
const int ESC_DEADZONE = 20;
const int ESC_MAX_REVERSE = 1000;
const int MAX_THROTTLE_US = 2000;

uint16_t front = 0, left = 0, right = 0;
const int SENSOR_IGNORE_DIST = 80;
const int RADAR_MIN_ANGLE = 55;
const int RADAR_MAX_ANGLE = 125;
#define RADAR_STEP 5
#define RADAR_POINTS ((RADAR_MAX_ANGLE - RADAR_MIN_ANGLE) / RADAR_STEP + 1)
int radarDistances[RADAR_POINTS];
int radarIndex = 0;
bool radarSweepForward = true;

const int AUTO_REVERSE_DURATION = 2000;
const int AUTO_THROTTLE_FORWARD = 1625;
const int AUTO_THROTTLE_REVERSE = 1380;
const int AUTO_STEER_ZONE_START = 450 + SENSOR_IGNORE_DIST;
const int AUTO_REVERSE_THRESHOLD = 150 + SENSOR_IGNORE_DIST;
const int AUTO_REVERSE_UNTILL = 250 + SENSOR_IGNORE_DIST;
const int AUTO_SENSOR_IGNORE_DISTANCE = 1000;
const int AUTO_MAX_STEER_ANGLE = 35;
const int AUTO_STEER_RIGHT_MAX = map(AUTO_MAX_STEER_ANGLE, 1, 55, STEER_STRAIGHT, STEER_RIGHT_MAX);
const int AUTO_STEER_LEFT_MAX = map(-AUTO_MAX_STEER_ANGLE, -55, 0, STEER_LEFT_MAX, STEER_STRAIGHT);

unsigned long lastRadarUpdate = 0;
const int radarUpdateInterval = 50;
const int distLogInterval = 500;
const int autonomousLogicUpdateInterval = 120;
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 20;

String driveMode = "manual";
int steerAngle = 0;
int autoAvoidDirection = 0;
bool isReversing = false;
bool sensorCrashed = false;
bool recoveringSensors = false;
bool wasAutoBeforeCrash = false;

void logMessage(String code, String msg = "") {
  logs += code + ":" + msg + ", ";
  Serial.println("[" + code + "] " + msg);
}

bool safeInit(VL53L0X &sensor, int maxRetries = 3)
{
  while (maxRetries--)
  {
    if (sensor.init(true))
      return true;
    delay(100);
  }
  return false;
}

void initSensor(VL53L0X &sensor, int resetPin, int newAddress)
{
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(100);
  digitalWrite(resetPin, HIGH);
  delay(300);
  Serial.println("Looking for new sensor at 0x29");
  Serial.println("Scanning...");
  for (byte addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("Found I2C at 0x");
      Serial.println(addr, HEX);
    }
  }
  if (safeInit(sensor))
  {
    logMessage("init sensor at : 0x" + String(newAddress, HEX) + " pin: " + String(resetPin));
    delay(10);
    sensor.setAddress(newAddress);
    sensor.setTimeout(30);
    sensor.startContinuous(33);
    delay(20);
  }
  else
  {
    logMessage("init sensor failed at : 0x" + String(newAddress, HEX) + " pin: " + String(resetPin));
  }
  delay(200);
}

uint16_t getLastRangeNonBlocking(VL53L0X &sensor)
{
  // Get the I2C address from the sensor instance
  uint8_t address = sensor.readReg(VL53L0X::I2C_SLAVE_DEVICE_ADDRESS);

  const uint8_t RESULT_RANGE_STATUS = 0x14;
  const uint8_t RANGE_HIGH_BYTE_OFFSET = 10;

  Wire.beginTransmission(address);
  Wire.write(RESULT_RANGE_STATUS + RANGE_HIGH_BYTE_OFFSET); // 0x1E
  if (Wire.endTransmission(false) != 0)
  {
    return 0xFFFF; // transmission error
  }

  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() < 2)
  {
    return 0xFFFF; // read error or not ready
  }

  uint16_t range = Wire.read() << 8;
  range |= Wire.read();

  return range;
}

void sensorReadTask()
{
  uint16_t f = getLastRangeNonBlocking(centerSensor);
  delayMicroseconds(100);
  uint16_t l = getLastRangeNonBlocking(leftSensor);
  delayMicroseconds(100);
  uint16_t r = getLastRangeNonBlocking(rightSensor);

  if (f == 0xFFFF || l == 0xFFFF || r == 0xFFFF)
  {
    sensorCrashed = true;
    return;
  }

  front = f;
  left = l;
  right = r;
  sensorCrashed = false;
}

void setup()
{
  Serial.begin(115200);
  esc.attach(escPin);
  steer.attach(steerPin);
  esc.writeMicroseconds(NEUTRAL);
  steer.write(STEER_STRAIGHT);
  sensorServo.attach(sensorServoPin);
  sensorServo.write(90);

  pinMode(centerSensorResetPin, OUTPUT);
  pinMode(leftSensorResetPin, OUTPUT);
  pinMode(rightSensorResetPin, OUTPUT);
  digitalWrite(centerSensorResetPin, LOW);
  digitalWrite(leftSensorResetPin, LOW);
  digitalWrite(rightSensorResetPin, LOW);
  Wire.begin();
  delay(300);
  Wire.setClock(100000);

  logMessage("[INFO] init sensors");
  initSensor(centerSensor, centerSensorResetPin, 0x30);
  initSensor(leftSensor, leftSensorResetPin, 0x31);
  initSensor(rightSensor, rightSensorResetPin, 0x32);

  driveMode = EEPROM.read(EEPROM_ADDR_MODE) == 1 ? "auto" : "manual";
  logMessage(LOG_MODE_SWITCH, driveMode);

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  logMessage("AP IP: " + ip.toString());
  server.begin();
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length)
                    {
    if (type == WStype_TEXT) {
      String msg = String((char*)payload);
      JSONVar json = JSON.parse(msg);
  
      if (JSON.typeof(json) == "undefined") {
        logMessage("[WARN] Received invalid JSON");
        return;
      }
  
      String type = json["type"];
      if (type == "ping") {
        webSocket.sendTXT(num, "{\"type\":\"pong\"}");
      } else if (type == "mode") {
        driveMode = (const char*)json["value"];
        if (driveMode == "manual") {
          esc.writeMicroseconds(NEUTRAL);
        } else if (driveMode == "auto") {
          radarIndex = 0;
          radarSweepForward = true;
        }
        EEPROM.write(EEPROM_ADDR_MODE, driveMode == "auto" ? 1 : 0);
        logMessage(LOG_MODE_SWITCH, driveMode);
      } else if (type == "input") {
        int t = (int)json["throttle"];
        int s = (int)json["steering"];
        handleThrottleInput(t);
        steerAngle = constrain(s, -55, 55);
        if (steerAngle <= 0) steer.write(map(steerAngle, -55, 0, STEER_LEFT_MAX, STEER_STRAIGHT));
        else steer.write(map(steerAngle, 1, 55, STEER_STRAIGHT + 1, STEER_RIGHT_MAX));
      }
    } });

  logMessage("[INFO] Boot complete");
}

unsigned long currentMillis = 0;

void loop()
{
  currentMillis = millis();
  webSocket.loop();

  WiFiClient client = server.available();
  if (client && client.connected())
  {
    String request = client.readStringUntil('\r');
    client.flush();
    if (request.indexOf("GET / ") >= 0)
    {
      sendWebUI(client);
    }
    client.stop();
  }

  if (currentMillis - lastSensorReadTime >= sensorReadInterval)
  {
    lastSensorReadTime = currentMillis;
    if (sensorCrashed)
    {
      if (driveMode == "auto")
      {
        wasAutoBeforeCrash = true;
        driveMode = "manual";
        esc.writeMicroseconds(NEUTRAL);
        logMessage(LOG_MODE_SWITCH, driveMode);
      }
      logMessage(LOG_SENSOR_FAIL);
      delay(100);
      NVIC_SystemReset();
    }
    else
    {
      sensorReadTask();
    }
  }

  static unsigned long lastAutoRadarUpdate = 0;
  if (currentMillis - lastAutoRadarUpdate >= radarUpdateInterval)
  {
    lastAutoRadarUpdate = currentMillis;
    updateRadarSweep();
  }

  if (driveMode == "auto")
  {
    static unsigned long lastAutonomousLogicTime = 0;
    if (currentMillis - lastAutonomousLogicTime >= autonomousLogicUpdateInterval)
    {
      lastAutonomousLogicTime = currentMillis;
      autonomousLogic();
    }
  }

  static unsigned long lastDistLogTime = 0;
  if (currentMillis - lastDistLogTime >= distLogInterval) {
    lastDistLogTime = currentMillis;
  
    JSONVar telemetry;
    telemetry["type"] = "telemetry";
    telemetry["front"] = front;
    telemetry["left"] = left;
    telemetry["right"] = right;
  
    // Send only log codes with optional payloads
    JSONVar logArray = JSONVar();
    int start = 0;
    while (true) {
      int end = logs.indexOf(", ", start);
      if (end == -1) break;
      String logEntry = logs.substring(start, end);
      int sep = logEntry.indexOf(":");
      if (sep > 0) {
        JSONVar entry;
        entry["code"] = logEntry.substring(0, sep);
        entry["msg"] = logEntry.substring(sep + 1);
        logArray[logArray.length()] = entry;
      }
      start = end + 2;
    }
    telemetry["logs"] = logArray;
  
    String jsonStr = JSON.stringify(telemetry);
    webSocket.broadcastTXT(jsonStr);
    logs = "";
  }
  
  unsigned long after = millis();
  Serial.println("time to loop " + String(after - currentMillis));
}

void updateRadarSweep()
{
  int angle = RADAR_MAX_ANGLE - radarIndex * RADAR_STEP; // flipped direction
  sensorServo.write(angle);
  if (front < SENSOR_IGNORE_DIST || front > AUTO_SENSOR_IGNORE_DISTANCE)
  {
    radarDistances[radarIndex] = AUTO_SENSOR_IGNORE_DISTANCE;
  }
  else
  {
    radarDistances[radarIndex] = front;
  }

  radarIndex += radarSweepForward ? 1 : -1;
  if (radarIndex >= RADAR_POINTS)
  {
    radarIndex = RADAR_POINTS - 1;
    radarSweepForward = false;
  }
  else if (radarIndex < 0)
  {
    radarIndex = 0;
    radarSweepForward = true;
  }
  // ✅ Send current radar point during sweep
  JSONVar radarPoint;
  radarPoint["type"] = "radar";
  radarPoint["angle"] = angle;
  radarPoint["distance"] = front;
  String txt = JSON.stringify(radarPoint);
  webSocket.broadcastTXT(txt);
}

int getRadarSectorAverage(int startIdx, int endIdx)
{
  int total = 0, count = 0;
  for (int i = startIdx; i <= endIdx; i++)
  {
    int d = radarDistances[i];
    if (d > SENSOR_IGNORE_DIST && d <= AUTO_SENSOR_IGNORE_DISTANCE)
    {
      total += d;
      count++;
    }
  }
  return (count > 0) ? (total / count) : AUTO_SENSOR_IGNORE_DISTANCE;
}

void autonomousLogic()
{
  int radarLeftAvg = getRadarSectorAverage(0, RADAR_POINTS / 3);
  int radarCenterAvg = getRadarSectorAverage(RADAR_POINTS / 3 + 1, 2 * RADAR_POINTS / 3);
  int radarRightAvg = getRadarSectorAverage(2 * RADAR_POINTS / 3 + 1, RADAR_POINTS - 1);

  // Static sensors take priority if available and closer than radar
  if (left > SENSOR_IGNORE_DIST && left < AUTO_SENSOR_IGNORE_DISTANCE)
  {
    radarLeftAvg = min(radarLeftAvg, left);
  }
  if (right > SENSOR_IGNORE_DIST && right < AUTO_SENSOR_IGNORE_DISTANCE)
  {
    radarRightAvg = min(radarRightAvg, right);
  }

  int leftAvg = radarLeftAvg;
  int centerAvg = radarCenterAvg;
  int rightAvg = radarRightAvg;

  int closest = min(centerAvg, min(leftAvg, rightAvg));

  if ((centerAvg < AUTO_REVERSE_THRESHOLD && centerAvg > SENSOR_IGNORE_DIST) || (isReversing && centerAvg < AUTO_REVERSE_UNTILL))
  {
    if (!isReversing)
    {
      esc.writeMicroseconds(NEUTRAL);
    }
    isReversing = true;
    steer.write(leftAvg < rightAvg ? AUTO_STEER_RIGHT_MAX : AUTO_STEER_LEFT_MAX);
    esc.writeMicroseconds(AUTO_THROTTLE_REVERSE);
    leftAvg < rightAvg ? logMessage(LOG_AUTO_REVERSE_RIGHT) : logMessage(LOG_AUTO_REVERSE_LEFT);
  }
  else if (closest < AUTO_STEER_ZONE_START && closest > SENSOR_IGNORE_DIST)
  {
    if (isReversing)
    {
      esc.writeMicroseconds(NEUTRAL);
    }
    isReversing = false;
    steer.write(leftAvg > rightAvg ? AUTO_STEER_LEFT_MAX : AUTO_STEER_RIGHT_MAX);
    esc.writeMicroseconds(AUTO_THROTTLE_FORWARD);
    leftAvg > rightAvg ? logMessage(LOG_AUTO_AVOID_RIGHT) : logMessage(LOG_AUTO_AVOID_LEFT);
  }
  else
  {
    if (isReversing)
    {
      esc.writeMicroseconds(NEUTRAL);
    }
    isReversing = false;
    steer.write(STEER_STRAIGHT);
    esc.writeMicroseconds(AUTO_THROTTLE_FORWARD);
    logMessage(LOG_AUTO_FORWARD);
  }
}

void handleThrottleInput(int value)
{
  int microVal = NEUTRAL;
  if (value != 0)
  {
    int absVal = abs(value);
    int throttlePercent = absVal <= 80 ? map(absVal, 1, 80, 1, 30) : map(absVal, 81, 100, 31, 100);
    microVal = value > 0 ? map(throttlePercent, 1, 100, NEUTRAL + 1, MAX_THROTTLE_US)
                         : map(throttlePercent, 1, 100, NEUTRAL - 1, ESC_MAX_REVERSE);
  }
  esc.writeMicroseconds(microVal);
}

extern const char index_html[]; // defined in client.html

void sendWebUI(WiFiClient &client)
{
  client.println("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.println(index_html);
}


int parseValue(String request, String key)
{
  int i = request.indexOf(key + "=") + key.length() + 1;
  int j = request.indexOf(" ", i);
  return request.substring(i, j).toInt();
}
