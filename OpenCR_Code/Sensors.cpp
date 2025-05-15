#include "Sensors.h"
#include "ros_functions.h"
//int ore_count_temp = 0;


int sensorPin = A5;
int sensorValue = 0;

void initSensors(){
  pinMode(PART_PRESENT_SESNOR_PIN, INPUT_PULLDOWN);  // Set pin 7 as input
  pinMode(HALL_EFFECT_SENSOR_PIN, INPUT);  // Set pin 8 as input
}

bool detectOre(){
	return !((bool) digitalRead(PART_PRESENT_SESNOR_PIN));
}
bool detectMag(){
	return (bool) digitalRead(HALL_EFFECT_SENSOR_PIN);
}

bool detectLed(){
	sensorValue = analogRead(sensorPin);
	if(sensorValue > LIGHT_TRESHOLD){
		//detectLedPub();
		pubScoopDone(true);
		return true;
	}
    Serial.println(sensorValue);
	return false;
}