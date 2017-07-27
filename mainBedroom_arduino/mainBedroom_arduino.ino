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
#define CLIENT_ID "client-mBedroom"
String mqttMessage = "";
//constants to update arduino status
const char* device_online = "online";
const char* device_offline = "offline";
const char* mqttTopic_deviceStatus = "/mBedroom/deviceStatus";
unsigned long device_lastTimeTaken;
#define deviceStatusDelay 60000

//set constants for stepperMotor
//sm1 short for stepperMotor 1
const int sm1_steps = 200; //steps in one revolution
const int sm1_rpm = 120;
#define sm1_enableA 12
#define sm1_enableB 13
unsigned int sm1_Enable = 0;
bool sm1_inUse = false;
unsigned long sm1_lastTimeEnabled;
#define smRestTime 30000 //time before disabling motor
Stepper sm1(sm1_steps, 8,9,10,11);

//set constants for DHT22 (temperature and humidity) sensor
#define DHTTYPE DHT22
#define DHTPIN 2
#define sensorDelay 60000 //time in milliseconds
#define sensorTDelay 4000 //for avgeraging purpuses
unsigned long dht_lastTimeTaken;
unsigned long currentTime;
unsigned long dht_tempTime;
float t_temp;
float h_temp;
unsigned int nSamples;
float t;
float h;
char t_char[6];
char h_char[6];
DHT dhtSensor(DHTPIN, DHTTYPE);


//----Functions for Stepper motors----
void spinMotor1(int steps, float revolutions){
  if(sm1_Enable==0){
    sm1_Enable =1;
    analogWrite(sm1_enableA, 255);
    analogWrite(sm1_enableB, 255);
    Serial.println("sm1 enabled");
  }
  sm1_inUse = true;
  sm1.step(steps*revolutions);
  sm1_inUse = false;
  sm1_lastTimeEnabled = millis();
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
    spinMotor1(sm1_steps, mqttMessage.toFloat()); 
  }
    
}

void reconnect() {
  // Loop until we're reconnected
  while (!client1.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client1.connect("Arduino", mqttTopic_deviceStatus, 0, 1, device_offline)) {
      Serial.println("connected");
      client1.publish(mqttTopic_deviceStatus,device_online, 1);
      device_lastTimeTaken = millis();
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
    sm1_lastTimeEnabled = millis();
    pinMode(sm1_enableA, OUTPUT);
    pinMode(sm1_enableA, OUTPUT);
    digitalWrite(sm1_enableA, LOW);
    digitalWrite(sm1_enableB, LOW);

    //setup DHT sensor
    dhtSensor.begin();
    dht_lastTimeTaken = millis();
    dht_tempTime = 0;
    nSamples = 0;
    h_temp = 0;
    t_temp = 0;
}

void loop() {
    //connecting to mqtt broker
    if(!client1.connected()){
      reconnect();
    }
    //mqtt listener
    client1.loop();

    
    currentTime = millis();

    //check how long steppermotor has been enabled
    if(sm1_Enable==1 && !sm1_inUse){
      if(abs(currentTime-sm1_lastTimeEnabled)>smRestTime){
        sm1_Enable=0;
        analogWrite(sm1_enableA, 0);
        analogWrite(sm1_enableB, 0);
        Serial.println("sm1 disabled");
      }
    }
    
    //collect sensor data for averaging
    if(abs(currentTime - dht_tempTime) > sensorTDelay){
      float h1 = dhtSensor.readHumidity();
      float t1 = dhtSensor.readTemperature();
      if (!isnan(h1) && !isnan(t1)){
        h_temp += h1;
        t_temp += t1;
        nSamples++;
        dht_tempTime = millis();
      }
    }

    //publishing sensor data
    if(abs(currentTime - dht_lastTimeTaken) > sensorDelay){
      h = h_temp/nSamples;
      t = t_temp/nSamples;
      h_temp = 0;
      t_temp = 0;
      nSamples = 0;
      if(isnan(h) || isnan(t)){
        Serial.println("Failed to read DHT sensor");
      }else{
        h_char;
        t_char;
        dtostrf(h, 4,2, h_char);
        dtostrf(t, 3,2, t_char);
        //char t_final[20] = "Temp:";
        //char h_final[20] = "Humidity:";
        //strcat(t_final, t_char);
        //strcat(h_final, h_char);
        client1.publish("/mBedroom/sensors/temperature", t_char, 1);
        client1.publish("/mBedroom/sensors/humidity", h_char, 1);
        dht_lastTimeTaken = millis();
      }
    }
 
}
