#include <Arduino.h>
const int ledPin = 12;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received

uint32_t xt_data[2] = {0x266e817d,0xbacd5035}; //2*32Bit Data


void setup() {

    pinMode(ledPin,OUTPUT);
    Serial.begin(9600);
    Serial.write("1234");
}

void loop() {

    digitalWrite(ledPin,HIGH);
    delay(1000);
    digitalWrite(ledPin,LOW);
    delay(1000);
    Serial.write("1234");
}
  
    

  


  
