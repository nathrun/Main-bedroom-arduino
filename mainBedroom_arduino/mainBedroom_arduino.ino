#include <SPI.h>
#include <Ethernet.h>
#include <Stepper.h>
#include <PubSubClient.h>
#include <DHT.h>

//set constants for the Ethernet on this Arduino
//mac address must be unique on local network
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10,0,0,21);
IPAddress myDns(10,0,0,2);
EthernetClient etherClient;

//set constants for mqtt broker and topics that this
//arduino must connect to
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

//set constants for DHT22 (temperature and humidity) sensor
#define DHTTYPE DHT22
#define DHTPIN 2
#define sensorDelay 60000 //time in milliseconds
unsigned long lastTimeTaken;
unsigned long currentTime;
float t;
float h;
char t_char[6];
char h_char[6];
DHT dhtSensor(DHTPIN, DHTTYPE);


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
      client1.publish("/house/deviceStatus","mBedroom-aurdiuno online");
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
    Serial.begin(4800);
    client1.setServer(mqttBrokerIP, 1883);
    client1.setCallback(callback);

    //setup for stepper motors
    sm1.setSpeed(sm1_rpm);

    //setup DHT sensor
    dhtSensor.begin();
    lastTimeTaken = millis();
}

void loop() {
    //connecting to mqtt broker
    if(!client1.connected()){
      reconnect();
    }
    //mqtt listener
    client1.loop();

    //publishing sensor data    --make sure this is at the end of loop()!
    currentTime = millis();
    
    if(abs(currentTime - lastTimeTaken) > sensorDelay){
      h = dhtSensor.readHumidity();
      t = dhtSensor.readTemperature();
      if(isnan(h) || isnan(t)){
        Serial.println("Failed to read DHT sensor");
      }
      h_char;
      t_char;
      dtostrf(h, 4,2, h_char);
      dtostrf(t, 3,2, t_char);
      //char t_final[20] = "Temp:";
      //char h_final[20] = "Humidity:";
      //strcat(t_final, t_char);
      //strcat(h_final, h_char);
      client1.publish("/mBedroom/sensors/temperature", t_char);
      client1.publish("/mBedroom/sensors/humidity", h_char);
      lastTimeTaken = millis();
    }
 
}
