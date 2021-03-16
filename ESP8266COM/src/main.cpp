#include <Arduino.h>
const uint16_t ledPin = 12;

const uint8_t numChars = 8;
char receivedChars[numChars];   // an array to store the received Data

bool newData = false;

uint32_t xt_data[2] = {0x266e817d,0xbacd5035}; //2*32Bit Data

void recvWithStartEndMarkers();
void showNewData(); 



void setup() {

    pinMode(ledPin,OUTPUT);
    Serial.begin(9600);
    Serial.println("ready");
}

void loop() {

    recvWithStartEndMarkers();
    showNewData();
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static uint8_t ndx = 0;
    char startMarker = '<';
    char endMarker = '\n';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
}
    

  


  
