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

unsigned long fin = 0;
float AngleX = 0.0;
float AngleStable = 95;

int Kp = 6;
int Ki = 0.01;
int Kd = 0.05;

float error = 0.0;
float d_error = 0;

float P = 0.0;
float I=0.0;
float D = 0.0;

float PID = 0.0;

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

  // Utilisation du PWM sur l'ESP32, car cette dernière ne dispose pas de broches analogiques.
  // On associe une broche GPIO numérique à un canal PWM.
  // Le PWM permet de simuler une sortie analogique à partir d'une broche numérique.
  // En ajustant le rapport HIGH/LOW (duty cycle) sur chaque cycle, et en répétant ce cycle à une fréquence élevée (ici 1000 Hz),
  // la broche de sortie fournit un signal perçu comme stable à la valeur moyenne souhaitée.


  //ledcSetup(channelPWM, fréquence, résolution) (la résolution donne la précision à laquelle on peut choisir la valeur de notre signal)
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcSetup(2, 1000, 8);
  ledcSetup(3, 1000, 8);

  //association des PINs et des channelPWM
  ledcAttachPin(IN_1_D, 0);
  ledcAttachPin(IN_2_D, 1);
  ledcAttachPin(IN_1_G, 2);
  ledcAttachPin(IN_2_G, 3);

}

void stopMotor() {

  ledcWrite(IN_1_D, 0);
  ledcWrite(IN_2_D, 0);
  ledcWrite(IN_1_G, 0);
  ledcWrite(IN_2_G, 0);

}

void move(float speed1, float speed2) {
  ledcWrite(0, speed1);
  ledcWrite(1, speed2);
  ledcWrite(2, speed2);
  ledcWrite(3, speed1);
} 



void loop () {
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
float angleAX = atan2(AccelY, sqrt(AccelX*AccelX + AccelZ*AccelZ))*360/PI; 

// Complementary filter coefficient to balance gyro and accelerometer
float alpha = 0.95;  

// Compute final angle by combining gyroscope integration and accelerometer angle
AngleX = alpha * AngleX + (1 - alpha) * angleAX;




error = AngleStable -  AngleX;


P = Kp*error;
I+=Ki*error*dt;
D=(error-d_error)/dt;
d_error=error;

PID = P+I+D;
PID = constrain(PID, -255, 255);
if(PID >= 0)
{
  move(PID,0);
}else if(PID < 0){
  move(0,-PID);
}



Serial.print(AngleX);
Serial.print(" | ");
Serial.print(angleAX);
Serial.print(" | ");
Serial.print(PID+50);
Serial.print(" | ");
Serial.println(-PID-50);


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