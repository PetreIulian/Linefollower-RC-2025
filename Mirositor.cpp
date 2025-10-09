#include "MotorShield.h"
#include "DRI0040_Motor.h"
#include "QTRSensors.h"

//Flags
#define DEBUG_FLAG false
#define CALIBRATION_FLAG true

bool robot_state = true;

#define MAXSPEED 100
#define SensorCount 8

//Motor Config
#define M1DIR  19
#define M1PWM  20
#define M2DIR  32
#define M2PWM  33
MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

//Sensor Config
QTRSensors qtr;
const uint8_t sensorPins[SensorCount] = {5, 8, 25, 26, 12, 15, 14, 13};
uint16_t sensorValues[SensorCount];

//PID values
double Kp = 0.05;
double Kd = 0.8;
double Ki = 0;


double baseSpeed = 25;
int lastError = 0;
double integral = 0;

int calculateError() {
  int position = qtr.readLineBlack(sensorValues); // 0 (left) -> 7000 (right)
  int error = position - 3500;
  return error;
}

//PID Function
double PID(int error) {
  integral += error;
  double derivative = error - lastError;
  lastError = error;

  double output = Kp * error + Ki * integral + Kd * derivative;
  return output;
}

//Calibration Function
void calibrate() {
  Serial.println("-----Calibration has started-----");
  delay(1000);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(10);
  }

  Serial.println("-----Calibration Completed");

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" max=");
    Serial.println(qtr.calibrationOn.maximum[i]);
  }
  delay(1000);
}

//Debug Function
void debug() {
  while (true) {
    uint16_t position = qtr.readLineBlack(sensorValues);

    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print("\t");
    }

    Serial.print("Pos: ");
    Serial.println(position);
    delay(400);
  }
}

void setup() {

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  delay(500);

  motors.begin();

  for (int i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("Robot ON");

  if (CALIBRATION_FLAG) {
    Serial.begin(115200);
    calibrate();
  }

  if (DEBUG_FLAG) {
    Serial.begin(115200);
    debug();
  }
}

void loop() {
  if (!robot_state) return;

  int error = calculateError();
  double correction = constrain(PID(error), -MAXSPEED, MAXSPEED);

  double left = baseSpeed - correction;
  double right = baseSpeed + correction;

  left = constrain(left, -MAXSPEED, MAXSPEED);
  right = constrain(right, -MAXSPEED, MAXSPEED);

  motors.setM1speed(left);
  motors.setM2speed(right);

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print("\tCorection: ");
  Serial.print(correction);
  Serial.print("\tL: ");
  Serial.print(left);
  Serial.print("\tR: ");
  Serial.println(right);

  delay(10);
}