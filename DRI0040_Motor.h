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
    DRI0040_Motor(uint8_t InPin1, uint8_t InPin2);

    void begin(){
      pinMode(InPin1, OUTPUT);
      pinMode(InPin2, OUTPUT);
    }

  void forward(int speed) {
    digitalWrite(InPin2, LOW);
    analogWrite(InPin1, speed);
  }

  void reverse(int speed) {
    digitalWrite(InPin1, LOW);
    analogWrite(InPin2, speed);
  }

  void stop() {
  	digitalWrite(InPin1, LOW);
    digitalWrite(InPin2, LOW);
  }

  void set_speed(int speedPercentage) {
    speedPercentage = constrain(speedPercentage, -100, 100);

    const int speed = map(abs(speedPercentage), 0, 100, 0, 255);

    if (speedPercentage > 0) {
        forward(speed);
    }
    else {
      reverse(-speed);
    }

  }

  private:
    uint8_t InPin1;
    uint8_t InPin2;
}
#endif //DRI0040_MOTOR_H
