#include <SPI.h>
#include <Ethernet.h>
#include <Stepper.h>
#include <PubSubClient.h>

//set constants for the Ethernet on this Arduino
//mac address must be unique on local network
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10,0,0,21);
IPAddress myDns(10,0,0,2);
EthernetClient etherClient;

//set constants for mqtt broker and topics that this
//Arduino must connect to
IPAddress mqttBrokerIP(10,0,0,14);
const char* mqttTopic1 = "/mBedroom/stepper";
PubSubClient client1(etherClient);
const char* mqttTopic2 = "/mBedroom/curtains";
#define CLIENT_ID "client-mainBedroom"
String mqttMessage = "";

//set constants for stepperMotor
//sm1 short for stepperMotor 1
const int sm1_steps = 200; //steps in one revolution
const int sm1_rpm = 120;
Stepper sm1(sm1_steps, 8,9,10,11);


//----Functions for Stepper motors----
void spinMotor(Stepper sm, int steps, float revolutions){
    sm.step(steps*revolutions);
}

//----Functions for mqtt----
void callback(char* topic, byte* payload, unsigned int length) {
  mqttMessage = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    mqttMessage.concat((char)payload[i]);
  }
  Serial.println();
  if (strcmp(topic, mqttTopic2)==0){
    spinMotor(sm1, sm1_steps, mqttMessage.toFloat()); 
  }
    
}

void reconnect() {
  // Loop until we're reconnected
  while (!client1.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client1.connect("Arduino")) {
      Serial.println("connected");
      client1.subscribe(mqttTopic1);
      client1.subscribe(mqttTopic2);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client1.state());
      Serial.println(" try again in 1 seconds");
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}


void setup() {
    //setup for Ethernet and mqtt
    Ethernet.begin(mac, ip);
    Serial.begin(9600);
    client1.setServer(mqttBrokerIP, 1883);
    client1.setCallback(callback);

    //setup for stepper motors
    sm1.setSpeed(sm1_rpm);
}

void loop() {
    if(!client1.connected()){
      reconnect();
    }

    client1.loop();
}
