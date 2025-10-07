#include "MotorShield.h"
#include "DRI0040_Motor.h"

#define DEBUG_FLAG true

#define MAXSPEED 100

//MotorShield
#define M1DIR  13
#define M1PWM  14
#define M2DIR  15
#define M2PWM  12
MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

bool robot_state = false;

//Sensor Config
#define SensorCount 13
const int sensorPins[SensorCount] = {19, 20, 32, 33, 36, 37, 38, 39, 21, 22, 5, 8, 25};

int sensorWeights[SensorCount] = {-600, -500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500, 600};

int sensorMin[SensorCount];
int sensorMax[SensorCount];
int sensorValues[SensorCount];

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
      if(digitalRead(sensorPins[SensorCount])) {
        weightSum += sensorWeights[i];
        activeCount++;
      }
    }

    if (activeCount != 0) {
      error = weightSum /(double)activeCount;
    }

    // adjustable field to toy with
    if (error < 0.1) {
      error = 0;
    }

    return error;
}

double PID(double error) {
   integral += error;
   double derivative = error - lastError;
   double PID = Kp * error + Kd * derivative + Ki * integral;
   lastError = error;

   return PID;

/* maybe :))
   const float alpha = 0.85;
   const float dAlpha = 0.875;

   integral += error;
   double base_derivative = error - lastError
   double derivative = dAlpha * base_derivative + (1 - Alpha) * last_derivative;
   lastError = error;

   double PID = (Kp * error) + (Kd * derivative) + (Ki * integral);
   return PID;
 */
}

void change_robot_state() {

}

/*maybe someday
void calibrate(){
   Serial.println("-----Sensor Calibration-----");

  for (int i = 0; i < SensorCount; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  unsigned long startTime = millis();
  while(millis() - startTime < 10000) {
     for (int i = 0; i < SensorCount; i++) {
      int value = analogRead(sensorPins[i]);
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
  }
  Serial.println("-----Calibration Complete-----");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" -> Min: ");
    Serial.print(sensorMin[i]);
    Serial.print("  Max: ");
    Serial.println(sensorMax[i]);
  }
}

void readSensors() {
  for (int i = 0; i < SensorCount; i++) {
    int value = analogRead(sensorPins[i]);
    int normalized = map(value, sensorMin[i], sensorMax[i], 0, 1000);
    normalized = constrain(normalized, 0, 1000);
    sensorValues[i] = normalized;
  }
}*/

void setup() {
  if(DEBUG_FLAG) {
    Serial.begin(115200);
    delay(4000);
  }

  motors.begin();

  for (int i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("Robot Activ");

}

void loop() {

   if(DEBUG_FLAG) {
    Serial.println("Debug has started:");

    loop_debug:
    Serial.println("\nSensor inputs:");
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(digitalRead(sensorPins[i]));
      Serial.print("\t");
    }
    Serial.print("\n-----------------------------------------------------------------------------------------------");
    delay(2000);
    goto loop_debug;
  }

  if(robot_state) {

  double error = calculateError();
  double correction = constrain(PID(error), -MAXSPEED, MAXSPEED);

  double left = baseSpeed - correction;
  double right = baseSpeed + correction;

  motors.setM1speed(left);
  motors.setM2speed(right);

  delay(10);
  }

}