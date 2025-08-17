#include <Arduino.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <math.h>


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
float OffiAngle = 0.0;


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

}

void loop () {
//Variation infinitésimale de temps (pour intégrer)
unsigned long debut = millis();
float dt = (debut - fin)/1000.0; 
fin = debut;

//(X car c'est l'axe qui nous intéresse) récupération de la vitesse angulaire du gyroscope
// + correction approximative de l'offset (+2,5°/s)
float degX =  imu.readFloatGyroX()- 2.5;

//valeur estimée de l'angle par le gyroscope (sur le court terme correct et dérive sur le long terme)
AngleX += degX*dt;

  delay(10);


//récuperation des valeurs de l'accéléromètre
  float AccelX = imu.readFloatAccelX()-0.02;
  float AccelY = imu.readFloatAccelY()+0.27;
  float AccelZ = imu.readFloatAccelZ()+0.97;

//passage en angle des valeurs de l'accéléromètre
  float angleAX = atan2(AccelY, (sqrt(AccelX*AccelX+AccelZ*AccelZ)))*180/PI; 

//implémentation d'un coefficient de "confiance" des 2 méthodes
float alpha = 0.95;  

//calcul final de l'angle avec regroupement des 2 méthodes
AngleX = alpha*(AngleX) + (1 - alpha)*angleAX;

Serial.println(AngleX);

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