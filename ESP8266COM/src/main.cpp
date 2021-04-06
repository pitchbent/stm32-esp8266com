#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <xtea.h>
#include <crc16ccitt.h>
#include <PubSubClient.h>

#define BAUD 115200

const char* ssid = "WuTangLAN";
const char* password = "11933793430981488313";
const char* mqtt_server = "192.168.178.68";

WiFiClient espClient;
PubSubClient client(espClient);


const uint16_t ledPin = 12;

const uint8_t numChars = 60;
char receivedChars[numChars];   // an array to store the received Data
bool newData = false;
uint16_t timeout = 10000;
static uint16_t length, length1;

uint32_t xt_data[2] = {0x266e817d,0xbacd5035}; //2*32Bit Data

char crc,crc1;



void receive();
void showNewData(); 

void setup_wifi();
void callback(String topic, byte* message, unsigned int length);
void reconnect();

void setup() {

    pinMode(ledPin,OUTPUT);
    Serial.begin(BAUD);
    Serial.println("ready");

    setup_wifi();
    client.setServer(mqtt_server,1883);
    client.setCallback(callback);



}

void loop() {

    receive();
    //showNewData();
    if (!client.connected()) {
    reconnect();
    }
    if(!client.loop())
    client.connect("ESP8266Client");
    //client.publish("testTopic","arschi");
    if (newData == true)
    {
    Serial.println("pub mq");
    client.publish("testTopic",receivedChars,length1);
    //client.publish("testTopic","arschi");
    newData = false;
    }
}

void receive()
{
  static boolean recvInProgress = false;
  static boolean beginRead = false;
  static boolean length_f, length_f1 = false;
  static byte ndx = 0;
  char startMarker = '<';
  char rc;
  static uint32_t timer;



  /*Check for timeout*/
  if (recvInProgress == true && (millis() - timer > timeout))
  {
    ndx = 0;
    recvInProgress = false;
    Serial.println("timeout");
  }

  /*Receive Data*/
  while (Serial.available() > 0 && newData == false)
  {

    timer = millis();   //store timer to detect time out
    rc = Serial.read();

    if (recvInProgress == true)
    {

      if (length_f == true && length_f1 == true) //The length bytes are stored - store incoming data
      {
        if (ndx < length1)
        {
          Serial.print("ndx count: ");
          Serial.println(ndx);
          receivedChars[ndx] = rc;
          Serial.println(receivedChars[ndx]);
        }
        if (ndx == length1 + 1) //store crc byte 2
        {
          crc1 = rc;
          recvInProgress = false;
          newData = true;
          Serial.print("rec chars: ");
          for (uint8_t i = 0; i < length1; i++)
          {
            Serial.print(receivedChars[i]);
          }
          Serial.println();
          Serial.print("crc values: ");
          Serial.print(crc);
          Serial.print(crc1);

          //Serial.println("crc1");
        }
        if (ndx == length1) //store crc byte 1
        {
          crc = rc;

          //Serial.println("crc");
        }

        ndx++; //increment array 
      }

      if (length_f == true && length_f1 == false) //Store length byte 2
      { 
        length1 = (rc - 48) + (length * 10); //calculate length
        length_f1 = true;
        Serial.println("length 1 is:");
        Serial.println(length1);
      }

      if (length_f == false && length_f1 == false) //Store length byte 1
      {
        length = rc - 48; //ASCII to int
        length_f = true;
        Serial.println("length  is:");
        Serial.println(length);
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;        
      length_f = false;             //Reset all the flags & counters
      length_f1 = false;
      ndx = 0;
      Serial.println("inprogress");
    }
  }
}

void showNewData() {
    if (newData == true) {
        digitalWrite(ledPin,HIGH);
        //Serial.print("<12345678");
        Serial.println(receivedChars);
        newData = false;
        for (uint8_t i = 0; i < numChars; i++)
        {
            receivedChars[i] = 0;
        }
        
    }
}
    

  
// Don't change the function below. This functions connects your ESP8266 to your router
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
// Change the function below to add logic to your program, so when a device publishes a message to a topic that 
// your ESP8266 is subscribed you can actually do something
void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
}

// This functions reconnects your ESP8266 to your MQTT broker
// Change the function below if you want to subscribe to more topics with your ESP8266 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

  
