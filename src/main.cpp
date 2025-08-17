#include <Arduino.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"


// User led
#define LEDU1 25
#define LEDU2 26
// Enable moteurs droit et gauche
#define EN_D 23
#define EN_G 4
// Commande PWM moteur droit
#define IN_1_D 19
#define IN_2_D 18
// Commande PWM moteur gauche
#define IN_1_G 17
#define IN_2_G 16
// Encodeur gauche
#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
// Encodeur droit
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14
// I2C
#define SDA 21
#define SCL 22
// Adresse I2C
#define ADDR_IMU 0x6B
#define ADDR_MAG 0x1E

unsigned long fin = 0;
float AngleX = 0.0;
float AngleY = 0.0;
float AngleZ = 0.0;


LSM6DS3 imu(I2C_MODE, 0x6B);   


void setup () {
  fin = millis();
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

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

}

void loop () {
float degX =  imu.readFloatGyroX()- 2.5;
float degY =  imu.readFloatGyroY()+ 2.5;
float degZ =  imu.readFloatGyroZ()+ 2.5;



/*
  Serial.print(imu.readFloatGyroX() - 2.5, 4);
  Serial.print(" | ");
  Serial.print(imu.readFloatGyroY() + 2.5, 4);
  Serial.print(" | ");
  Serial.println(imu.readFloatGyroZ() + 2.5, 4);*/

  delay(100);

  unsigned long debut = millis();
  float dt = (debut - fin)/1000.0;
  fin = debut;

  AngleX += degX*dt;
  AngleY += degY*dt;
  AngleZ += degZ*dt;

Serial.print(AngleX);
Serial.print(" | ");
Serial.print(AngleY);
Serial.print(" | ");
Serial.println(AngleZ);

}