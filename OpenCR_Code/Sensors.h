#include "macros.h"

#define PART_PRESENT_SESNOR_PIN 11
#define HALL_EFFECT_SENSOR_PIN 8
void reset_ore_count();
void initSensors();
bool detectOre();
bool detectMag();
bool detectLed();