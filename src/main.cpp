#include "gyro.hpp"

LSM6DS3 imu(I2C_MODE, 0x6B);  
unsigned long fin = 0;
float AngleStable = 98.3;

float Kp = 6.3, Ki = 0.1, Kd = 0.21;
float motorOffset = 53;

float error = 0.0;
float d_error = 0;

float P = 0.0;
float I=0.0;
float D = 0.0;

float D_filtre =0.0;

float PID = 0.0;
float AngleX = 0.0;

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
float alpha = 0.96;  

// Compute final angle by combining gyroscope integration and accelerometer angle
AngleX = alpha * AngleX + (1 - alpha) * angleAX;


  error = AngleStable - AngleX;

  P = Kp * error;

  I += Ki * error * dt;
  I = constrain(I, -25, 25); 

static float prev_error = 0;
D = Kd * (error - prev_error)/dt; 
prev_error = error;

D_filtre = 0.9 * D_filtre + 0.1 * D;

PID = P+I+D_filtre;
PID = constrain(PID, -255, 255);


  PID = constrain(P + I + D, -255, 255);

  // Commande moteur sécurisée
  int output = abs(PID) + motorOffset;
  output = constrain(output, 0, 255);  // Limitation manuelle du PWM

  if (PID >= 0) {
    move(output, 0);  // Moteur droit avance, gauche stop
  } else {
    move(0, output);  // Moteur gauche avance, droit stop
  } 



Serial.print("Objectif : ");
Serial.print(AngleStable);
Serial.print("Angle : ");
Serial.print(AngleX);
Serial.print("PID : ");
Serial.println(output);

Serial.print(">Objectif : ");
Serial.println(AngleStable);
Serial.print(">Angle : ");
Serial.println(AngleX);
Serial.print(">PID : ");
Serial.println(output);
Serial.print(">P : ");
Serial.println(P);
Serial.print(">D : ");
Serial.println(D);

}