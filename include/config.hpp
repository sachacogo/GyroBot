#pragma once

#include <Arduino.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <math.h>
#include <Adafruit_Sensor.h>

// Baud sur lequel parle l'ESP (à utiliser dans le Serial.begin())
constexpr unsigned long SERIAL_BAUD = 115200;

// Adresses de l'IMU et du Magnétoscope
constexpr int ADDR_IMU = 0x6B;
constexpr int ADDR_MAG = 0x1E;

// Coefficient du gyro dans le filtre complémentaire
constexpr float COMP_ALPHA = 0.95f;

extern float dt;

float getDt();