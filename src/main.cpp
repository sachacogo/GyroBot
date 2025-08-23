#include "pins.hpp"
#include "imu.hpp"
#include "config.hpp"
#include "motors.hpp"
#include "debugger.hpp"

LSM6DS3 imu(I2C_MODE, 0x6B);   


void setup () {
  Serial.begin(SERIAL_BAUD);
  Wire.begin(SDA_PIN, SCL_PIN);
  initIMU(imu);
  initMotors();
}

void loop () {
  roll_x = getRoll(imu);
  float pid = getPID(roll_x);
  stabilize(pid);

  //debugAll();
}