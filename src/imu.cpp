#include "imu.hpp"
#include "config.hpp"


unsigned long fin = 0;
float roll_x = 0.0f;

void initIMU(LSM6DS3 &imu) {
    imu.settings.gyroRange = 245;
    imu.settings.accelRange = 2;

    if (imu.begin() != 0) {
        Serial.print("Error at beginCore().\n");
    } else {
        Serial.print("\nbeginCore() passed.\n");
    }
}

float getRoll(LSM6DS3 &imu){
    unsigned long debut = millis();
    float dt = (debut - fin) / 1000.0;
    fin = debut;

    // Estimation de l'angle selon le gyroscope
    float GyroX = imu.readFloatGyroX() - GYR_X_BIAS;
    roll_x += GyroX * dt;

    //Estimation de l'angle selon l'accéléromètre
    float AccelX = imu.readFloatAccelX() - ACC_X_BIAS;
    float AccelY = imu.readFloatAccelY() - ACC_Y_BIAS;
    float AccelZ = imu.readFloatAccelZ() - ACC_Z_BIAS;

    float accRoll = atan2(AccelY, sqrt(AccelX*AccelX + AccelZ*AccelZ)) * 360 / PI;

    // Compensation de l'angle estimé par le gyroscope par le filtre complémentaire
    roll_x = COMP_ALPHA * roll_x + (1 - COMP_ALPHA) * accRoll;
    return roll_x;
}