#include <Arduino.h>

#include <main.h>

#include <ESP8266WiFi.h>
#include <crc16ccitt.h>
#include <PubSubClient.h>


const char* ssid = "yourWIFIhere";
const char* password = "yourPASSWORDhere";
const char* mqtt_server = "192.168.178.68";

String clientId = "ESP8266_Humid";

WiFiClient espClient;
PubSubClient client(espClient);

const uint16_t ledPin = 12;






uint8_t receivedChars[BUF_SIZE];   // an array to store the received Data
bool newData = false;
uint16_t length_pub;               // length of the mqtt msg

uint8_t mqtt_in_data[BUF_SIZE];





void receive();


void setup_wifi();
void callback(String topic, byte* message, unsigned int length);
void reconnect();

void setup() {

    pinMode(ledPin,OUTPUT);
    Serial.begin(BAUD);
    //Serial.println("ready");

    setup_wifi();
    client.setServer(mqtt_server,1883);
    client.setCallback(callback);



}

void loop() {

    receive();
    if (!client.connected()) {
      reconnect();
    }
    client.loop();                //check for msg, calls callback if new msg is available
//    if(!client.loop())
//      client.connect("ESP8266Client");
    if (newData == true)
    {
    //Serial.println("pub mq");
    //  for (uint8_t i = 0; i <= 6; i++)
    //  {
    //    Serial.print(receivedChars[i]);
    //  }
    
      client.publish("Humid",receivedChars+3,length_pub);   //offset by three to skip the start and length bytes 
      newData = false;
    }
}


void receive()
{
  static uint16_t count = 0;
  static bool InProg = false;
  static uint8_t len= 0;
  uint16_t crc_calc;
  static uint32_t tim;
  
  /*Check for timeout*/
  if (InProg == true && (millis() - tim > TIMEOUT))
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
        len = (receivedChars[count]-OFF_ASCII)+((receivedChars[count-1]-OFF_ASCII)*10);
      }

      if(InProg == true && count == len+4) //len +4 -> the second crc byte was stored
      {
        //crc = ((receivedChars[count-1] << 8)| receivedChars[count]);  //concatenate the two crc bytes

        crc_calc = CRC16_buf(receivedChars,count+1);  //count +1 because it gets iterated at the end
        //Serial.print("Calced CRC value: ");
        //Serial.println(crc_calc,HEX);
        if(crc_calc == 0)
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

  
// Connect to wifi
void setup_wifi() {
  bool led_flag = 0;
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (led_flag==0)
    {
      digitalWrite(ledPin,HIGH);
      led_flag = 1;
    }
    else
    {
      digitalWrite(ledPin,LOW);
      led_flag = 0;
    }
  }
  digitalWrite(ledPin,LOW);
}
//Callback if a message is published on the subscribed topic
//Converts the received message into a message which is understood by STM and forwards it
void callback(String topic, byte* message, unsigned int length) {
  char len_buf[2];
  uint16_t crc16;
 
  // Serial.print("Message arrived on topic: ");
  // Serial.print(topic);
  // Serial.print(". Message: ");
  
  
  sprintf(len_buf,"%d",length); //turn int to char array

  // Serial.println("len_buf items:");
  // Serial.println(len_buf[0]);
  // Serial.println(len_buf[1]);


  mqtt_in_data[0]=START_MARKER; 

  if (length <10)
  {
    // Serial.println("length under 10");
    mqtt_in_data[1] = '0';
    mqtt_in_data[2] = len_buf[0];
  }
  else
  {
    // Serial.println("length over 10");
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

  //Serial.println("Message to send:");

  for (uint8_t c = 0; c <= (length+4); c++)
  {
    Serial.print((char)mqtt_in_data[c]);        //cast to char so ASCII gets put out
  }
  
  
  


  
  
}

// This functions reconnects your ESP8266 to your MQTT broker
// Change the function below if you want to subscribe to more topics with your ESP8266 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    //String clientId = "ESP8266Client-";
    //clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      //Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("Humid", "Sensor connected!");
      // ... and resubscribe
      client.subscribe("SensorSetup");
      digitalWrite(ledPin,LOW);
    } else {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      digitalWrite(ledPin,HIGH);
      delay(5000);
    }
  }
}

  
