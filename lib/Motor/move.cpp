#include "move.hpp"
#include <Arduino.h>

// Set motor PWM commands
void move(float speed1, float speed2) {
  ledcWrite(0, speed1);
  ledcWrite(1, speed2);
  ledcWrite(2, speed2);
  ledcWrite(3, speed1);
} 