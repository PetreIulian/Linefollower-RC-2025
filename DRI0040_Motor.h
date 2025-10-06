//
// Created by Petre Ioan Iulian on 10/6/2025.
//

#ifndef DRI0040_MOTOR_H
#define DRI0040_MOTOR_H

#pragma once
#include <Arduino.h>
#include <stdint.h>

class DRI0040_Motor {
  public:
    DRI0040_Motor(uint8_t Dir, uint8_t PWM, bool invert = false);

    void begin(){
      pinMode(Dir, OUTPUT);
      pinMode(PWM, OUTPUT);
      stop();
    }

  void stop() {
  	analogWrite(Dir, 0);
  }

  void set_speed(int speedPercentage) {
    speedPercentage = constrain(speedPercentage, -100, 100);

    const int speed = map(abs(speedPercentage), 0, 100, 0, 255);
    const bool forward = (speedPercentage < 0);

    digitalWrite(Dir, (forward ^ invert) ? LOW : HIGH)

  }

  private:
    uint8_t Dir;
    uint8_t PWM;
    bool invert;
}
#endif //DRI0040_MOTOR_H
