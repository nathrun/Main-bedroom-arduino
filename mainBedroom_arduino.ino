#include <SPI.h>
#include <Ethernet.h>
#include <Stepper.h>
#include <PubSubClient.h>

//set constants for the Ethernet on this Arduino
//mac address must be unique on local network
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(xxx, xxx, xxx, xxx);
IPAddress myDns(xxx,xxx,xxx, xxx);
EthernetClient etherClient;

//set constants for mqtt broker and topics that this
//Arduino must connect to
IPAddress mqttBrokerIP = "xxx.xxx.xxx.xxx";
const char* mqttTopic1 = "/test/stepperMotor";
#define CLIENT_ID "client-mainBedroom"
PubSubClient client(etherClient);

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
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
/*
  // Examine only the first character of the message
  if(payload[0] == 49)              // Message "1" in ASCII (turn outputs ON)
  {
    digitalWrite(ledPin, LOW);      // LED is active-low, so this turns it on
    digitalWrite(relayPin, HIGH);
  } else if(payload[0] == 48)       // Message "0" in ASCII (turn outputs OFF)
  {
    digitalWrite(ledPin, HIGH);     // LED is active-low, so this turns it off
    digitalWrite(relayPin, LOW);
  } else {
    Serial.println("Unknown value");
  }
*/
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(CLIENT_ID)) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}


void setup() {
    //setup for Ethernet and mqtt
    Ethernet.begin(mac, ip);
    Serial.begin(9600);
    client.setServer(mqttBrokerIP, 1883);
    client.setCallback(callback);

    //setup for stepper motors
    sm1.setSpeed(sm1_rpm);
}

void loop() {
    if(!client.connected()){

    }
}
