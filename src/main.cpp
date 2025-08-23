#include "pins.hpp"
#include "imu.hpp"
#include "config.hpp"



LSM6DS3 imu(I2C_MODE, 0x6B);   


void setup () {
  Serial.begin(SERIAL_BAUD);
  Wire.begin(SDA_PIN, SCL_PIN);
  initIMU(imu);

  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);

  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);

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
  
  Serial.println(getRoll(imu));

}