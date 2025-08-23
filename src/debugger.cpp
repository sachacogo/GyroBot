#include "debugger.hpp"

void debugAll() {
    Serial.print("Objectif : ");
    Serial.print(STABLE_ANGLE);
    Serial.print("Angle : ");
    Serial.print(roll_x);
    Serial.print("PID : ");
    Serial.println(output);

    Serial.print(">Angle : ");
    Serial.println(roll_x);
    Serial.print(">P : ");
    Serial.println(P);
    Serial.print(">I : ");
    Serial.println(I);
    Serial.print(">D : ");
    Serial.println(D);
    Serial.print(">PID : ");
    Serial.println(PID);
    Serial.print(">Consigne : ");
    Serial.println(STABLE_ANGLE);
}