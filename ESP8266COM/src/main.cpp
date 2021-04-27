#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <xtea.h>
#include <crc16ccitt.h>
#include <uartcom.h>
#include <PubSubClient.h>

#define BAUD 115200
#define START_MARKER '<'

const char* ssid = "WuTangLAN";
const char* password = "11933793430981488313";
const char* mqtt_server = "192.168.178.68";

WiFiClient espClient;
PubSubClient client(espClient);

const uint16_t ledPin = 12;

const uint8_t numChars = 99;
uint8_t receivedChars[numChars];   // an array to store the received Data
bool newData = false;
uint16_t timeout = 10000;
uint16_t length_pub;

uint8_t mqtt_in_data[numChars];




//uint32_t xt_data[2] = {0x266e817d,0xbacd5035}; //2*32Bit Data






void receive();


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
    if (!client.connected()) {
    reconnect();
    }
    if(!client.loop())
    client.connect("ESP8266Client");
    if (newData == true)
    {
    Serial.println("pub mq");
    client.publish("testTopic",receivedChars+3,length_pub);
    client.publish("testTopic","test");
   
    newData = false;
    }
}


void receive()
{
  static uint16_t count = 0;
  static bool InProg = false;
  static uint8_t len= 0;
  uint16_t crc, crc_calc;
  static uint32_t tim;
  
  /*Check for timeout*/
  if (InProg == true && (millis() - tim > timeout))
  {
    count = 0;
    InProg = false;
  }

  while (Serial.available() > 0 && newData == false)
  {
      
      tim = millis(); //Update timer
      receivedChars[count] = Serial.read(); //Read serial

      if (receivedChars[count] == START_MARKER) //Detect start
      {
        InProg = true;
      }


      if(InProg == true && count == 2) //2 -> the second length byte was stored
      {
        len = (receivedChars[count]-48)+((receivedChars[count-1]-48)*10);
      }

      if(InProg == true && count == len+4) //len +4 -> the second crc byte was stored
      {
        crc = ((receivedChars[count-1] << 8)| receivedChars[count]);  //concatenate the two crc bytes

        crc_calc = CRC16_buf(receivedChars,count-1);  //ignore last two bytes

        if(crc == crc_calc)
        {
          length_pub = len; //pass the count
          InProg = false; //reset
          newData = true; //New data is available
          len = 0;        //reset
          count = 0;      //reset
 
          return;
        }
        else
        {
          InProg = false;   //reset
          newData = false;  //New data is not available
          len = 0;          //reset
          count = 0;        //reset 
          
          return;         
        }
        
      }

        count ++; 



  
  }
  


}

  
// Don't change the function below. This functions connects your ESP8266 to your router
void setup_wifi() {

  delay(10);
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
//Callback if a message is published on the subscribed topic
//Converts the received message into a message which is understood by STM and forwards it
void callback(String topic, byte* message, unsigned int length) {
  char len_buf[2];
  uint16_t crc16;
 
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  
  sprintf(len_buf,"%d",length); //turn int to char array

  Serial.println("len_buf items:");
  Serial.println(len_buf[0]);
  Serial.println(len_buf[1]);


  mqtt_in_data[0]=START_MARKER; 

  if (length <10)
  {
    Serial.println("length under 10");
    mqtt_in_data[1] = '0';
    mqtt_in_data[2] = len_buf[0];
  }
  else
  {
    Serial.println("length over 10");
    mqtt_in_data[1] = len_buf[0];
    mqtt_in_data[2] = len_buf[1];
  }
  
  
  //mqtt_in_data[1]=len_buf[0];   //append the length bytes
  //mqtt_in_data[2]=len_buf[1];

  for (uint8_t i = 0; i < length; i++)    //append the msg
  {
    mqtt_in_data[i+3] = message[i];
  }

  crc16 = CRC16_buf(mqtt_in_data,length+3); //calc the crc16

  mqtt_in_data[length+3] = (crc16 >> 8) & 0xFF; //append crc16 split into two bytes
  mqtt_in_data[length+4] = (crc16 >> 0) & 0xFF;

  Serial.println("Message to send:");

  for (uint8_t c = 0; c <= (length+4); c++)
  {
    Serial.print(mqtt_in_data[c],HEX);
  }
  
  
  


  
  
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
      client.subscribe("testTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

  
