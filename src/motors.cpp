#include "motors.hpp"

float P = 0.0f;
float I = 0.0f;
float D = 0.0f;
float PID = 0.0f;
float error = 0.0f;
float prev_error = 0.0f;
int output = 0;

void initMotors() {
    pinMode(EN_D, OUTPUT);
    pinMode(EN_G, OUTPUT);

    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);

    pinMode(IN_1_D, OUTPUT);
    pinMode(IN_2_D, OUTPUT);
    pinMode(IN_1_G, OUTPUT);
    pinMode(IN_2_G, OUTPUT);

    // Utilisation du PWM sur l'ESP32, car cette dernière ne dispose pas de broches analogiques.
    // On associe une broche GPIO numérique à un canal PWM.
    // Le PWM permet de simuler une sortie analogique à partir d'une broche numérique.
    // En ajustant le rapport HIGH/LOW (duty cycle) sur chaque cycle, et en répétant ce cycle à une fréquence élevée (ici 1000 Hz),
    // la broche de sortie fournit un signal perçu comme stable à la valeur moyenne souhaitée.
    ledcSetup(0, 1000, 8);
    ledcSetup(1, 1000, 8);
    ledcSetup(2, 1000, 8);
    ledcSetup(3, 1000, 8);

    // Association des PINs et des channelPWM
    ledcAttachPin(IN_1_D, 0);
    ledcAttachPin(IN_2_D, 1);
    ledcAttachPin(IN_1_G, 2);
    ledcAttachPin(IN_2_G, 3);
}

void move(float speed1, float speed2) {
    ledcWrite(0, speed1);
    ledcWrite(1, speed2);
    ledcWrite(2, speed2);
    ledcWrite(3, speed1);
}

float getPID(float roll_x) {
    error = STABLE_ANGLE - roll_x;
    P = Kp * error;
    I += Ki * error * dt;
    I = constrain(I, -25, 25);
    D = Kd * (error - prev_error) / dt;
    prev_error = error;
    D = constrain(D, -255, 255);

    PID = P + I + D;
    PID = constrain(PID, -255, 255);
    return PID;
}

void stabilize(float pid) {
    output = abs(pid) + motorOffset;
    output = constrain(output, 0, 255);  // Limitation manuelle du PWM

    if (pid > 0) {
        move(output, 0);  // Moteur droit avance, gauche stop
    } else {
        move(0, output);  // Moteur gauche avance, droit stop
    }
}