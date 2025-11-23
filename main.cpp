#include "MotorShield.h"
#include "DRI0040_Motor.h"
#include "QTRSensors.h"
#include "BluetoothSerial.h"
#include <EEPROM.h>

//Flags
#define DEBUG_FLAG false
#define CALIBRATION_FLAG true

bool robot_state = false;

#define MAXSPEED 90
#define MAX_OUTPUT 40
#define SensorCount 8

//Motor Config
#define M1DIR  20
#define M1PWM  19
#define M2DIR  33
#define M2PWM  32
MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

//Sensor Config
QTRSensors qtr;
const uint8_t sensorPins[SensorCount] = {5, 8, 25, 26, 15, 12, 14, 13};
uint16_t sensorValues[SensorCount];

//PID values
double KP = 0.01315; // to do lower
double KD = 0.175; // to do higher
double KI = 0.00085;

unsigned long startTime = 0;
unsigned long accelerationTime = 3000; 

int baseSpeed = 0, setBaseSpeed = 40;
double line_position = 0;
double lastError = 0;
double integral = 0;
double error = 0, error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0;

BluetoothSerial SerialBT;

void start_stop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();

    if (cmd == 'S' || cmd == 's') {
      robot_state = true;
    } 
    else if (cmd == 'P' || cmd == 'p') {
      robot_state = false;
    }
  }
}

double calculateError() {
  line_position = qtr.readLineBlack(sensorValues); // 0 (left) -> 7000 (right)
  error = 4250 - line_position;
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
  double output = KP * error + KD * derivative + KI * integral;
  return output;
}

//Calibration Function
void calibrate() {
  
  if(Serial){
    Serial.println("-----Calibration has started-----");
  }

  delay(1000);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(10);
  }

  if(Serial){
    Serial.println("-----Calibration Completed-----");

    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": min=");
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(" max=");
      Serial.println(qtr.calibrationOn.maximum[i]);
   }

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

//Function for saving the calibration on EEPROM
void saveCalibration() {
  int adrr = 0;
  int minVal = 0;
  int maxVal = 0;


  for(int i = 0; i <= SensorCount; i++) {
    minVal = qtr.calibrationOn.minimum[i];
    maxVal = qtr.calibrationOn.maximum[i];

    EEPROM.write(minVal, adrr);
    adrr += sizeof(uint16_t);

    EEPROM.write(maxVal, adrr);
    adrr += sizeof(uint16_t); 
  }

  EEPROM.commit();
}

//Function for loading the calibration from EEPROM
void loadCalibration() {
  int adrr = 0;

  for(int i = 0; i <= SensorCount; i++) {
    qtr.calibrationOn.minimum[i] = EEPROM.read(adrr);
    adrr += sizeof(uint16_t);

    qtr.calibrationOn.maximum[i] = EEPROM.read(adrr);
    adrr += sizeof(uint16_t);

  }
}


void setup() {
  EEPROM.begin(524);

  while(!SerialBT.begin("ESP32_LineFollower")){

  }

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  delay(500);

  motors.begin();

  for (int i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("Robot ON");
  SerialBT.println("Bluetooth ready. Send 'S' to start or 'P' to stop.");

  if (CALIBRATION_FLAG) {
    calibrate(); 
    saveCalibration();
    SerialBT.println("------Calibration Saved------");
  } 
  else {
    loadCalibration();
    SerialBT.println("------Calibration Loaded------");
  }

  if (DEBUG_FLAG) {
    Serial.begin(115200); 
    debug();
  }
}

void loop() {
  start_stop();

  if (startTime == 0 && robot_state) {
    startTime = millis();
  }

  unsigned long elapsedTime = millis() - startTime;

  if (elapsedTime <= accelerationTime) {
    baseSpeed = map(elapsedTime, 0, accelerationTime, 0, setBaseSpeed);
  }

  error = calculateError();
  double correction = 0;

  correction = constrain(PID(error), -MAX_OUTPUT, MAX_OUTPUT);
  
  double left = baseSpeed - correction;
  double right = baseSpeed + correction;


  if ((line_position > 0 && line_position <= 1000) || (line_position >= 6000 && line_position < 7000)) {
    baseSpeed = 0.85 * setBaseSpeed;
  } else {
    baseSpeed = setBaseSpeed;
  }

  left = constrain(left, 0, MAXSPEED);
  right = constrain(right, 0, MAXSPEED);

  if (robot_state) {
    motors.setM1speed(left);
    motors.setM2speed(right);

    if (Serial) {
      Serial.print("Error: "); Serial.print(error);
      Serial.print("\tCorrection: "); Serial.print(correction);
      Serial.print("\tL: "); Serial.print(left);
      Serial.print("\tR: "); Serial.println(right);
    }

  } else {
    motors.setM1speed(0);
    motors.setM2speed(0);
    startTime = 0;
  }

  delay(5);
}
