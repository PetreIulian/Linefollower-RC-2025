#include "MotorShield.h"
#include "DRI0040_Motor.h"

#define MAXSPEED 100

//MotorShield
#define M1DIR  13
#define M1PWM  14
#define M2DIR  15
#define M2PWM  12
MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

//Sensor Config
#define SensorCount 13
const int sensorPins = {};

int sensorWeights[SensorCount] = {-600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 600};

//PID Config
double Kp = 0;
double Kd = 0;
double Ki = 0;

double baseSpeed = 25;
double error = 0, lastError = 0, integral = 0;

double calculateError () {
    double weightSum = 0;
    short int activeCount = 0;

    for (int i = 0; i < SensorCount; i++) {
      if(digitalRead(sensorPins)) {
        weightSum += sensorWeights[i];
        activeCount++;
      }
    }

    if (activeCount != 0) {
      error = weightSum /(float)activeCount;
    }

    // adjustable field to toy with
    if (error < 0.1) {
      error = 0;
    }
}

void setup() {


}

void loop() {


}