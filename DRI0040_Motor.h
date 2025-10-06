#ifndef DRI0040_MOTOR_H
#define DRI0040_MOTOR_H

#pragma once
#include <Arduino.h>

class DRI0040_Motor {
public:
  DRI0040_Motor(uint8_t Dir, uint8_t PWM, bool invert = false)
    : Dir(Dir), PWM(PWM), invert(invert) {}

  void begin() {
    pinMode(Dir, OUTPUT);
    pinMode(PWM, OUTPUT);
    stop();
  }

  void stop() {
    digitalWrite(Dir, LOW);    
    analogWrite(PWM, 0);
  }

  void set_speed(int speedPercentage) {
    speedPercentage = constrain(speedPercentage, -100, 100);

    const int speed = map(abs(speedPercentage), 0, 100, 0, 255);
    const bool forward = (speedPercentage > 0);

    digitalWrite(Dir, (forward ^ invert) ? LOW : HIGH);

    analogWrite(PWM, speed);
  }

private:
  uint8_t Dir;    
  uint8_t PWM;    
  bool invert;    
};

#endif // DRI0040_MOTOR_H