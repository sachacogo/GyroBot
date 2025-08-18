#include "pins.hpp"
#include "imu.hpp"
#include "config.hpp"
#include <Arduino.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <math.h>
#include <Adafruit_Sensor.h>



float AngleX = 0.0;

LSM6DS3 imu(I2C_MODE, 0x6B);   


void setup () {
  fin = millis();
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  imu.settings.gyroRange = 245;
  imu.settings.accelRange = 2;


  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);

  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);

  if( imu.begin() != 0 )
  {
    Serial.print("Error at beginCore().\n");
  }
  else
  {
    Serial.print("\nbeginCore() passed.\n");
  }

  //utilisation de la méthode du pwm car il n'y a pas de pin analogique sur une esp32
  //on doit prendre une PIN digitale et lui associer un PWM
  //le PWM est une méthode qui sert à transformer une PIN digitale en une PIN analogique
  //en faisant varier sur 1 cycle le rapport HIGH/LOW et en calculant la moyenne sur ce cycle

  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcSetup(2, 1000, 8);
  ledcSetup(3, 1000, 8);

  ledcAttachPin(IN_1_D, 0);
  ledcAttachPin(IN_2_D, 1);
  ledcAttachPin(IN_1_G, 2);
  ledcAttachPin(IN_2_G, 3);

  

}

void loop () {
/*
// Compute elapsed time for integration (in seconds)
unsigned long debut = millis();
float dt = (debut - fin)/1000.0; 
fin = debut;

// Read angular velocity from gyroscope (X axis is relevant)
// Apply approximate offset correction (+2.5°/s)
float degX =  imu.readFloatGyroX() - 2.5;

// Update estimated angle from gyroscope (accurate short-term, drifts long-term)
AngleX += degX * dt;

  delay(10);


// Read accelerometer measurements
float AccelX = imu.readFloatAccelX() - 0.02;
float AccelY = imu.readFloatAccelY() + 0.27;
float AccelZ = imu.readFloatAccelZ() + 0.97;

// Convert accelerometer readings to tilt angle (degrees between -180 and 180)
float angleAX = atan2(AccelY, sqrt(AccelX*AccelX + AccelZ*AccelZ)) * 360 / PI; 

// Complementary filter coefficient to balance gyro and accelerometer
float alpha = 0.95;  

// Compute final angle by combining gyroscope integration and accelerometer angle
AngleX = alpha * AngleX + (1 - alpha) * angleAX;
*/
Serial.println(getRoll(imu));




/*
//SEPARATION VLAD
float gyroX = imu.readFloatGyroX();
float gyroY = imu.readFloatGyroY();
float gyroZ = imu.readFloatGyroZ();

float accX = imu.readFloatAccelX();
float accY = imu.readFloatAccelY();
float accZ = imu.readFloatAccelZ();

float phiAcc = atan(accY/(sqrt(accX*accX+accZ*accZ)))*180/PI;
float phiGyr;
phiGyr += gyroX * dt;

Serial.print(phiAcc);
Serial.print(" | ");
Serial.println(phiGyr);
*/
}