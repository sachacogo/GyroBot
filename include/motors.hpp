#pragma once
#include <Arduino.h>
#include "SparkFunLSM6DS3.h"

constexpr int motorOffset = 53;
constexpr float Kp = 6.3f;
constexpr float Ki = 0.0001f;
constexpr float Kd = 0.21f;

extern float P;
extern float I;
extern float D;
extern float PID;
extern float error;

void initMotors();
void move(float speed1, float speed2);
float getPID(float roll_x);