#include "fireSensorArray.h"

FireSensorArray::FireSensorArray(){
	for(int i=0; i<NUMFIRESENSORS; i++){
		fireSensePins[i] = -1;
	}
}

void FireSensorArray::attach(int pins[NUMFIRESENSORS]){
	for(int i=0; i<NUMFIRESENSORS; i++){
		fireSensePins[i] = pins[i];
		pinMode(fireSensePins[i],INPUT);
	}
}

void FireSensorArray::flip(){
	int tempPins[NUMFIRESENSORS];
	for(int i=1; i <= NUMFIRESENSORS; i++){
		tempPins[NUMFIRESENSORS-i] = fireSensePins[i-1];
	}
	for(int i=0; i < NUMFIRESENSORS; i++){
		fireSensePins[i] = tempPins[i];
	}
}

boolean FireSensorArray::isThereFire(){
	return (fireStrength()>FIRETHRESHOLD);
}

int FireSensorArray::fireAngle(){
  long curMax = 0;
  int curMaxI = -1;
  long weightedAngle = 0;
  long total = 0;
  int nSamples = 10;
  
  //Find max sensor to drive angle measurement
  for(int i = 0; i < NUMFIRESENSORS; i++) {
    long reading = 0;
    reading = analogRead(fireSensePins[i]);
    if(reading > curMax) {
      curMax = reading;
      curMaxI = i;
    }
  }
  
  for (int i=curMaxI-1; i<=curMaxI+1; i++)
  {
    if (i>=0 && i<NUMFIRESENSORS){
        long reading = 0;
        reading = analogRead(fireSensePins[i]);
        Serial.println(reading);
        long sensorAngle = map(i,0,NUMFIRESENSORS-1,60,-60);
        long weight = (100*reading);
        weight /= curMax; //Percentage of maximum reading
        total += weight;
        weightedAngle += (sensorAngle * weight);
    }
  }
  if (total == 0) total = 1;
  weightedAngle /= total;
  //if(curMax<FIRETHRESHOLD) weightedAngle=0; //No fire to read, don't give noise back as data
  return weightedAngle;
}

int FireSensorArray::fireStrength(){
  int curMax = 0;
  for(int i = 0; i < NUMFIRESENSORS; i++) {
    int reading = analogRead(fireSensePins[i]);
    if(reading > curMax) {
      curMax = reading;
    }
  }
  return curMax;
}

