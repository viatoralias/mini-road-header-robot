/*
Basic Test - Verify I2C communication and servo movement
*/
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("Basic I2C Scanner");
  scanI2C();
}

void loop() {
  // Simple heartbeat
  static int counter = 0;
  if (millis() % 1000 == 0) {
    Serial.print("System alive: ");
    Serial.println(counter++);
  }
}

void scanI2C() {
  byte error, address;
  int found = 0;
  
  Serial.println("Scanning I2C bus...");
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    }
  }
  
  if (found == 0) {
    Serial.println("No I2C devices found!");
  }
}
