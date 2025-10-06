//
// Created by Petre Ioan Iulian on 10/6/2025.
//

#ifndef DRI0040__MOTORSHIELD_H
#define DRI0040__MOTORSHIELD_H

#pragma once
#include <Arduino.h>
#include "DRI0040_Motor.h"

class DRI0040_MotorShield {
  public:
    DRI0040_MotorShield(uint8_t M1In1, uint8_t M1In2,
                        uint8_t M2In1, uint8_t M2In2)
    :M1 (M1In1, M1In2),
     M2 (M2In1, M2In2) {}

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
