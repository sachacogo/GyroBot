#include "pins.hpp"
#include "imu.hpp"
#include "config.hpp"
#include "motors.hpp"


LSM6DS3 imu(I2C_MODE, 0x6B);   


void setup () {
  Serial.begin(SERIAL_BAUD);
  Wire.begin(SDA_PIN, SCL_PIN);
  initIMU(imu);
  initMotors();
}

void loop () {

  Serial.println(getRoll(imu));

}