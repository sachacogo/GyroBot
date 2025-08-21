#include <Arduino.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <math.h>
#include <Adafruit_Sensor.h>

// User LEDs
#define LEDU1 25
#define LEDU2 26
// Enable right and left motors
#define EN_D 23
#define EN_G 4
// PWM control pins for right motor
#define IN_1_D 19
#define IN_2_D 18
// PWM control pins for left motor
#define IN_1_G 17
#define IN_2_G 16
// Left encoder channels
#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
// Right encoder channels
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14
// I2C pins
#define SDA 21
#define SCL 22
// I2C addresses
#define ADDR_IMU 0x6B
#define ADDR_MAG 0x1E
