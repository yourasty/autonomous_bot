#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X centerSensor, leftSensor, rightSensor;
Servo radarServo;

const int radarServoPin = 6;
const int resetPinCenter = 5;
const int resetPinLeft = 4;
const int resetPinRight = 3;

const int RADAR_RIGHT_ANGLE = 55;
const int RADAR_LEFT_ANGLE = 125;
const int RADAR_STEP = 5;
const int RADAR_INTERVAL = 25;
const int RADAR_POINTS = ((RADAR_LEFT_ANGLE - RADAR_RIGHT_ANGLE) / RADAR_STEP + 1);

uint16_t sweepDistances[RADAR_POINTS];
int sweepAngles[RADAR_POINTS];
int radarIndex = 0;
bool sweepingForward = true;

unsigned long lastRadarMove = 0;

void resetSensor(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(10);
  digitalWrite(pin, HIGH);
  delay(100);
}

void initSensor(VL53L0X& sensor, int resetPin, uint8_t address) {
  resetSensor(resetPin);
  sensor.setTimeout(50);
  sensor.init(true);
  delay(10);
  sensor.setAddress(address);
  sensor.startContinuous();
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  radarServo.attach(radarServoPin);
  radarServo.write(90);

  initSensor(centerSensor, resetPinCenter, 0x30);
  initSensor(leftSensor, resetPinLeft, 0x31);
  initSensor(rightSensor, resetPinRight, 0x32);
}

void loop() {
  // Only step radar and collect point at interval
  if (millis() - lastRadarMove > RADAR_INTERVAL) {
    lastRadarMove = millis();

    int logicalIndex = sweepingForward ? radarIndex : (RADAR_POINTS - 1 - radarIndex);
    int angle = RADAR_RIGHT_ANGLE + logicalIndex * RADAR_STEP;
    radarServo.write(angle);
    sweepAngles[logicalIndex] = angle;
    sweepDistances[logicalIndex] = centerSensor.readRangeContinuousMillimeters();

    radarIndex++;
    if (radarIndex >= RADAR_POINTS) {
      radarIndex = 0;
      sweepingForward = !sweepingForward;
      sendSweep();
    }
  }
}

void sendSweep() {
  uint16_t l = leftSensor.readRangeContinuousMillimeters();
  uint16_t r = rightSensor.readRangeContinuousMillimeters();

  Serial.print(l); Serial.print(",");
  Serial.print(r); Serial.print(":");

  for (int i = 0; i < RADAR_POINTS; i++) {
    Serial.print(sweepAngles[i]); Serial.print(":");
    Serial.print(sweepDistances[i]);
    if (i < RADAR_POINTS - 1) Serial.print("|");
  }
  Serial.println();
}
