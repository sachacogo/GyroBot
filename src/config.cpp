#include <Arduino.h>
#include "config.hpp"

float dt = millis();

float getDt() {
    unsigned long debut = millis();
    dt = (debut - dt) / 1000.0;
    dt = debut;
    return dt;
}