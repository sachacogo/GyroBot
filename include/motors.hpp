#pragma once
#include "config.hpp"
#include "pins.hpp"

constexpr int motorOffset = 53;
constexpr float Kp = 6.3f;
constexpr float Ki = 0.0001f;
constexpr float Kd = 0.21f;

extern float P;
extern float I;
extern float D;
extern float PID;

extern float error;
extern float prev_error;
extern int output; 

void initMotors();
void move(float speed1, float speed2);
float getPID(float roll_x);
void stabilize(float pid);