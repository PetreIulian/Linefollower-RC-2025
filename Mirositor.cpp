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

int sensorWeights = {};

//PID Config
double Kp = 0;
double Kd = 0;
double Ki = 0;

double baseSpeed = 25;
double error = 0, lastError = 0, integral = 0;
void setup() {


}

void loop() {


}