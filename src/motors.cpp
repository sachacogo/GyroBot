#include "motors.hpp"
#include "pins.hpp"
#include "config.hpp"

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
