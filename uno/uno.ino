#include <Arduino.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <WebSocketsServer.h>
#include <Arduino_JSON.h>
#include "client.h"

#define LOG_MODE_SWITCH "MODE_SWITCH"

String logs = "";

const char ssid[] = "RC_Controller";
const char pass[] = "";
WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

Servo esc, steer;
const int escPin = 9;
const int steerPin = 5;

const int STEER_STRAIGHT = 85;
const int MAX_STEER_ANGLE_LEFT = 50;
const int MAX_STEER_ANGLE_RIGHT = 50;
const int STEER_RIGHT_MAX = STEER_STRAIGHT + MAX_STEER_ANGLE_RIGHT;
const int STEER_LEFT_MAX = STEER_STRAIGHT - MAX_STEER_ANGLE_LEFT;
const int NEUTRAL = 1500;
const int ESC_MAX_REVERSE = 1000;
const int MAX_THROTTLE_US = 2000;

uint16_t left = 0, right = 0;
const int SENSOR_OFFSET = 30;
const int STATIC_SENSOR_OFFSET = 60;
#define RADAR_LEFT_ANGLE 125
#define RADAR_RIGHT_ANGLE 55
#define RADAR_STEP 5
#define RADAR_POINTS ((RADAR_LEFT_ANGLE - RADAR_RIGHT_ANGLE) / RADAR_STEP + 1)
int radarDistances[RADAR_POINTS];

const int AUTO_THROTTLE_FORWARD = 1625;
const int AUTO_THROTTLE_REVERSE = 1380;
const int AUTO_STEER_ZONE_START = 500 + SENSOR_OFFSET;
const int AUTO_REVERSE_THRESHOLD = 200 + SENSOR_OFFSET;
const int AUTO_REVERSE_UNTILL = 300 + SENSOR_OFFSET;
const int AUTO_SENSOR_IGNORE_DISTANCE = 1000;
const int AUTO_MAX_STEER_ANGLE = 35;
const int AUTO_STEER_RIGHT_MAX = map(AUTO_MAX_STEER_ANGLE, 1, 55, STEER_STRAIGHT, STEER_RIGHT_MAX);
const int AUTO_STEER_LEFT_MAX = map(-AUTO_MAX_STEER_ANGLE, -55, 0, STEER_LEFT_MAX, STEER_STRAIGHT);

unsigned long lastDistLogTime = 0;
const int distLogInterval = 500;
const int autonomousLogicUpdateInterval = 120;

String driveMode = "manual";
int steerAngle = 0;
bool isReversing = false;

unsigned long currentMillis = 0;
String serialBuffer = "";

void logMessage(String code, String msg = "")
{
  logs += code + ":" + msg + ", ";
  Serial.println("[" + code + "] " + msg);
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);

  esc.attach(escPin);
  steer.attach(steerPin);
  esc.writeMicroseconds(NEUTRAL);
  steer.write(STEER_STRAIGHT);

  logMessage(LOG_MODE_SWITCH, driveMode);

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  logMessage("AP IP: " + ip.toString());
  delay(1);
  server.begin();
  delay(1);
  webSocket.begin();
  delay(1);
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length)
  {
    if (type == WStype_TEXT) {
      String msg = String((char*)payload);
      JSONVar json = JSON.parse(msg);
      if (JSON.typeof(json) == "undefined") return;
      String type = json["type"];
      if (type == "ping") {
        webSocket.sendTXT(num, "{\"type\":\"pong\"}");
      } else if (type == "mode") {
        driveMode = (const char*)json["value"];
        if (driveMode == "manual") {
          esc.writeMicroseconds(NEUTRAL);
        }
        logMessage(LOG_MODE_SWITCH, driveMode);
      } else if (type == "input") {
        int t = (int)json["throttle"];
        int s = (int)json["steering"];
        handleThrottleInput(t);
        steerAngle = constrain(s, -55, 55);
        if (steerAngle <= 0) steer.write(map(steerAngle, -55, 0, STEER_LEFT_MAX, STEER_STRAIGHT));
        else steer.write(map(steerAngle, 1, 55, STEER_STRAIGHT + 1, STEER_RIGHT_MAX));
      }
    } 
  });

  logMessage("[INFO] Boot complete");
}

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
      sendWebUI(client);
    delay(1);
    client.stop();
  }
  int serialLoopMax = 1000;
  while (Serial1.available() && serialLoopMax > 0)
  {
    char c = Serial1.read();
    if (c == '\n')
    {
      handleSensorPacket(serialBuffer);
      serialBuffer = "";
    }
    else
    {
      serialBuffer += c;
    }
    serialLoopMax--;
  }

  if (driveMode == "auto")
  {
    static unsigned long lastAutoLogic = 0;
    if (currentMillis - lastAutoLogic > autonomousLogicUpdateInterval)
    {
      lastAutoLogic = currentMillis;
      autonomousLogic();
    }
  }

  if (currentMillis - lastDistLogTime >= distLogInterval)
  {
    lastDistLogTime = currentMillis;
    JSONVar telemetry;
    telemetry["type"] = "telemetry";
    telemetry["front"] = radarDistances[RADAR_POINTS / 2];
    telemetry["left"] = left;
    telemetry["right"] = right;

    JSONVar logArray;
    int start = 0;
    while (true)
    {
      int end = logs.indexOf(", ", start);
      if (end == -1)
        break;
      String logEntry = logs.substring(start, end);
      int sep = logEntry.indexOf(":");
      if (sep > 0)
      {
        JSONVar entry;
        entry["code"] = logEntry.substring(0, sep);
        entry["msg"] = logEntry.substring(sep + 1);
        logArray[logArray.length()] = entry;
      }
      start = end + 2;
    }
    telemetry["logs"] = logArray;

    JSONVar radarArr;
    for (int i = 0; i < RADAR_POINTS; i++)
    {
      int safeValue = constrain(radarDistances[i], 0, AUTO_SENSOR_IGNORE_DISTANCE);
      radarArr[i] = safeValue;
    }
    telemetry["radar"] = radarArr;

    String jsonStr = JSON.stringify(telemetry);
    webSocket.broadcastTXT(jsonStr);
    logs = "";
  }
}

void handleSensorPacket(String raw)
{
  // Expected format: l,r,a0:d0|a1:d1|...
  int l = raw.substring(0, raw.indexOf(',')).toInt();
  int r = raw.substring(raw.indexOf(',') + 1, raw.indexOf(':')).toInt();

  left = max(0, l - STATIC_SENSOR_OFFSET);
  right = max(0, r - STATIC_SENSOR_OFFSET);

  String sweepData = raw.substring(raw.indexOf(':') + 1);
  for (int i = 0; i < RADAR_POINTS; i++)
  {
    int sep = sweepData.indexOf('|');
    String pair = sep != -1 ? sweepData.substring(0, sep) : sweepData;
    int colon = pair.indexOf(':');
    if (colon > 0)
    {
      int a = pair.substring(0, colon).toInt();
      int d = pair.substring(colon + 1).toInt();
      int idx = (a - RADAR_RIGHT_ANGLE) / RADAR_STEP;
      if (idx >= 0 && idx < RADAR_POINTS)
        radarDistances[idx] = d;
    }
    if (sep == -1)
      break;
    sweepData = sweepData.substring(sep + 1);
  }
}

int getRadarSectorAverage(int startIdx, int endIdx)
{
  int total = 0, count = 0;
  for (int i = startIdx; i <= endIdx; i++)
  {
    int d = radarDistances[i];
    if (d > SENSOR_OFFSET && d <= AUTO_SENSOR_IGNORE_DISTANCE)
    {
      total += d;
      count++;
    }
  }
  return (count > 0) ? (total / count) : AUTO_SENSOR_IGNORE_DISTANCE;
}

int getRadarSectorMin(int startIdx, int endIdx) {
  int minVal = AUTO_SENSOR_IGNORE_DISTANCE;
  for (int i = startIdx; i <= endIdx; i++) {
    int d = radarDistances[i];
    if (d > SENSOR_OFFSET && d <= AUTO_SENSOR_IGNORE_DISTANCE) {
      minVal = min(minVal, d);
    }
  }
  return minVal;
}

void autonomousLogic()
{
  int radarRightAvg = getRadarSectorAverage(0, RADAR_POINTS / 3);
  int radarCenterAvg = getRadarSectorMin(RADAR_POINTS / 3 + 1, 2 * RADAR_POINTS / 3);
  int radarLeftAvg = getRadarSectorAverage(2 * RADAR_POINTS / 3 + 1, RADAR_POINTS - 1);

  if (left > SENSOR_OFFSET && left < AUTO_SENSOR_IGNORE_DISTANCE)
    radarLeftAvg = min(radarLeftAvg, left);
  if (right > SENSOR_OFFSET && right < AUTO_SENSOR_IGNORE_DISTANCE)
    radarRightAvg = min(radarRightAvg, right);

  int leftAvg = radarLeftAvg;
  int centerAvg = radarCenterAvg;
  int rightAvg = radarRightAvg;
  int closest = min(centerAvg, min(leftAvg, rightAvg));

  if ((centerAvg < AUTO_REVERSE_THRESHOLD) ||
      (isReversing && centerAvg < AUTO_REVERSE_UNTILL))
  {
    if (!isReversing) esc.writeMicroseconds(NEUTRAL);

    isReversing = true;
    steer.write(leftAvg > rightAvg ? AUTO_STEER_RIGHT_MAX : AUTO_STEER_LEFT_MAX);
    esc.writeMicroseconds(AUTO_THROTTLE_REVERSE);
  }
  else if (closest < AUTO_STEER_ZONE_START)
  {
    if (isReversing) esc.writeMicroseconds(NEUTRAL);

    isReversing = false;
    steer.write(leftAvg > rightAvg ? AUTO_STEER_LEFT_MAX : AUTO_STEER_RIGHT_MAX);
    esc.writeMicroseconds(AUTO_THROTTLE_FORWARD);
  }
  else
  {
    if (isReversing)
      esc.writeMicroseconds(NEUTRAL);
    isReversing = false;
    steer.write(STEER_STRAIGHT);
    esc.writeMicroseconds(AUTO_THROTTLE_FORWARD);
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

extern const char index_html[];
void sendWebUI(WiFiClient &client)
{
  client.println("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.println(index_html);
}
