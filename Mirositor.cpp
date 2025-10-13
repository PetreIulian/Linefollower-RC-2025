#include "MotorShield.h"
#include "DRI0040_Motor.h"
#include "QTRSensors.h"
#include <IRremote.h>

//Flags
#define DEBUG_FLAG false
#define CALIBRATION_FLAG true

bool robot_state = true;
#define IR_PIN 36

#define MAXSPEED 100
#define MAXOUTPUT 50
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
double Kp = 0.1;
double Kd = 0.8;
double Ki = 0;


double baseSpeed = 25;
double lastError = 0;
double integral = 0;
double error = 0, error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;

void start_stop() {
   if (IrReceiver.decode()) {
        uint8_t cmd = IrReceiver.decodedIRData.command;

        if (cmd == 0x45) {
            robot_state = 1;
        } else if (cmd == 0x46) {
            robot_state = 0;
        } else if (cmd == 0x47) {
            robot_state = 1;
        }

        IrReceiver.resume();
     }
}

double calculateError() {
  double position = qtr.readLineBlack(sensorValues); // 0 (left) -> 7000 (right)
  error = 3500 - position;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;
  return error;
}

//PID Function
double PID(int error) {
  integral = error6 + error5 + error4 + error3 + error2 + error1 + error;
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

  Serial.println("-----Calibration Completed-----");

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

  if (CALIBRATION_FLAG && Serial) {
    Serial.begin(115200);
  }
  else if (CALIBRATION_FLAG) {
    calibrate();
  }

  if (DEBUG_FLAG) {
    Serial.begin(115200);
    debug();
  }

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  if (!robot_state) return;

  error = calculateError();
  double correction = constrain(PID(error), -MAXOUTPUT, MAXOUTPUT);

  double left = baseSpeed - correction;
  double right = baseSpeed + correction;

  left = constrain(left, -MAXSPEED, MAXSPEED);
  right = constrain(right, -MAXSPEED, MAXSPEED);

  motors.setM1speed(left);
  motors.setM2speed(right);

  if(Serial) {
    Serial.print("Error: "); Serial.print(error);
    Serial.print("\tCorection: "); Serial.print(correction);
    Serial.print("\tL: "); Serial.print(left);
    Serial.print("\tR: "); Serial.println(right);
  }

  delay(10);
}