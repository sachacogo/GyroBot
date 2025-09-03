#include <Arduino.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <math.h>
#include <Adafruit_Sensor.h>


// Pin definitions
#define LEDU1 25
#define LEDU2 26
// Motor enable pins right & left
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

// IMU object LSM6DS3 accelerometer + gyroscope
LSM6DS3 imu(I2C_MODE, 0x6B);  

unsigned long fin = 0;
float AngleStable = 98.3;   // Experimentally determined equilibrium angle

// PID gains
float Kp = 6.3, Ki = 0.0001, Kd = 0.21;
float motorOffset = 53;     // Minimum PWM needed to overcome motor dead zone

// PID state variables
float error = 0.0;
float d_error = 0;
float P = 0.0;
float I = 0.0;
float D = 0.0;
float PID = 0.0;
float AngleX = 0.0;

void setup () {
  fin = millis();
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  // Configure IMU sensitivity
  imu.settings.gyroRange = 245;
  imu.settings.accelRange = 2;

  // Enable motor drivers
  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  // Motor PWM pins
  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);

  // Initialize IMU
  if ( imu.begin() != 0 ) {
    Serial.print("Error at beginCore().\n");
  } else {
    Serial.print("\nbeginCore() passed.\n");
  }

  // Configure PWM channels (0–3) with 1 kHz frequency, 8-bit resolution
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcSetup(2, 1000, 8);
  ledcSetup(3, 1000, 8);

  // Link PWM channels to output pins
  ledcAttachPin(IN_1_D, 0);
  ledcAttachPin(IN_2_D, 1);
  ledcAttachPin(IN_1_G, 2);
  ledcAttachPin(IN_2_G, 3);
}

// Set motor PWM commands
void move(float speed1, float speed2) {
  ledcWrite(0, speed1);
  ledcWrite(1, speed2);
  ledcWrite(2, speed2);
  ledcWrite(3, speed1);
} 

void loop () {
 
  //Time step for integrations
  unsigned long debut = millis();
  float dt = (debut - fin) / 1000.0;  
  fin = debut;


    // Gyroscope integration
    // Gyro X axis is relevant for balancing
    float degX = imu.readFloatGyroX() - 2.5; // Preliminary gyro offset correction
    AngleX += degX * dt;  

    delay(10);


    //Accelerometer measurement
    //With pre-filter accelerometer offset adjustment  
    float AccelX = imu.readFloatAccelX() - 0.02;
    float AccelY = imu.readFloatAccelY() + 0.27;
    float AccelZ = imu.readFloatAccelZ() + 0.97;

  // Convert accelerometer data into tilt angle
  float angleAX = atan2(AccelY, sqrt(AccelX*AccelX + AccelZ*AccelZ)) * 360 / PI; 

  // Complementary filter: combines gyro (short-term accuracy) and accel (long-term stability)
  float alpha = 0.96;  
  AngleX = alpha * AngleX + (1 - alpha) * angleAX;


  // PID control
  error = AngleStable - AngleX;

  // Proportional term
  P = Kp * error;

  // Integral term with anti-windup
  I += Ki * error * dt;
  I = constrain(I, -25, 25); 

  // Derivative term
  static float prev_error = 0;
  D = Kd * (error - prev_error) / dt; 
  prev_error = error;
  D = constrain(D, -255, 255);

  // PID output
  PID = P + I + D;
  PID = constrain(PID, -255, 255);

  // Motor command

  int output = abs(PID) + motorOffset;   // add offset to overcome motor dead zone
  output = constrain(output, 0, 255);    // clamp to valid PWM range

  if (PID >= 0) {
    move(output, 0);  // forward right motor
  } else {
    move(0, output);  // forward left motor
  } 


  //Teleplot view of main signals  
  Serial.print("Objectif : ");
  Serial.print(AngleStable);
  Serial.print("Angle : ");
  Serial.print(AngleX);
  Serial.print("PID : ");
  Serial.println(output);

  Serial.print(">Angle : ");
  Serial.println(AngleX);
  Serial.print(">P : ");
  Serial.println(P);
  Serial.print(">I : ");
  Serial.println(I);
  Serial.print(">D : ");
  Serial.println(D);
  Serial.print(">PID : ");
  Serial.println(PID);
  Serial.print(">Consigne : ");
  Serial.println(AngleStable);
}
