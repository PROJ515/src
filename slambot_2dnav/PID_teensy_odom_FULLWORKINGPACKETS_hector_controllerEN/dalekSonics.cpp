  //dalekSonics.cpp

#include "Arduino.h"

float duration;                                                          //Stores duration of pulse in
float distance;                                                        // Stores distance

float getRangeSRF(byte sensorpin){

  pinMode(sensorpin, OUTPUT);
  digitalWrite(sensorpin, LOW);                          // Make sure pin is low before sending a short high to trigger ranging
  delayMicroseconds(2);
  digitalWrite(sensorpin, HIGH);                         // Send a short 10 microsecond high burst on pin to start ranging
  delayMicroseconds(10);
  digitalWrite(sensorpin, LOW);                                  // Send pin low again before waiting for pulse back in
  pinMode(sensorpin, INPUT);
  duration = pulseIn(sensorpin, HIGH);                        // Reads echo pulse in from SRF05 in micro seconds
  distance = duration/5800;     // Dividing this by 5800 gives us a distance in meters
  return distance;
  
}
 

