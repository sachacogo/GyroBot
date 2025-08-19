#pragma once
#include <Arduino.h>
#include "SparkFunLSM6DS3.h"


// Offsets des capteurs
constexpr float ACC_X_BIAS = 0.02f;
constexpr float ACC_Y_BIAS = -0.27f;
constexpr float ACC_Z_BIAS = -0.97f;

constexpr float GYR_X_BIAS = 2.5f;

extern float roll_x;

// Décla des fonctions
void initIMU(LSM6DS3 &imu);
float getRoll(LSM6DS3 &imu);