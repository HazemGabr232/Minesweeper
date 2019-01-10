#include <Arduino.h>
#include <MetalDetector.h>

MetalDetector::MetalDetector(int pin) {
  this->pin = pin;
  pinMode(pin, INPUT);
}

int MetalDetector::detect() {
  return pulseIn(pin,HIGH,350000);  // returns 0 if there is a mine 
}
