//
// Created by Petre Ioan Iulian on 10/6/2025.
//

#ifndef DRI0040__MOTORSHIELD_H
#define DRI0040__MOTORSHIELD_H

#pragma once
#include <Arduino.h>
#include "DRI0040_Motor.h"

class MotorShield {
  public:
    MotorShield(uint8_t M1Dir, uint8_t M1PWM,
                uint8_t M2Dir, uint8_t M2PWM,
                bool M1invert, bool M2invert = false)
    : M1 (M1Dir, M1PWM, M1invert),
      M2 (M2Dir, M2PWM, M2invert) {}

   void begin() {
     M1.begin();
     M2.begin();
   }

   void setM1speed(int speed) {
     M1.set_speed(speed);
   }

   void setM2speed(int speed) {
     M2.set_speed(speed);
   }

   void stop() {
     M1.stop();
     M2.stop();
   }

  private: DRI0040_Motor M1;
           DRI0040_Motor M2;
};
#endif //DRI0040__MOTORSHIELD_H
