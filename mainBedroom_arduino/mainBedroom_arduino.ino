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
PubSubClient client1(etherClient);
#define CLIENT_ID "client-mBedroom"
String mqttMessage = "";
//constants to update arduino status
const char* device_online = "online";
const char* device_offline = "offline";
const char* mqttTopic_deviceStatus = "devices/mBedroom/arduino1Status";

//set constants for stepperMotor
//sm1 short for stepperMotor 1
//for curtains
const int sm1_steps = 200; //steps in one revolution
const int sm1_rpm = 120;
#define sm1_enableA 12
#define sm1_enableB 13
unsigned int sm1_Enable = 0;
bool sm1_inUse = false;
unsigned long sm1_lastTimeEnabled;
#define smRestTime 10000 //time before disabling motor
Stepper sm1(sm1_steps, 8,9,10,11);
const char* mqttCurtainsCommand = "mBedroom/curtains/command";
//set constants for TCRT
const int sm1_LimitPins[2] = {A0,A1};    //index 0 is for the sensor at closed, 1 at open
const int sm1_tcrtThreshold[2] = {80,80}; 


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

DHT dhtSensor(DHTPIN, DHTTYPE);



void setup() {
    //setup for Ethernet and mqtt
    Ethernet.begin(mac, ip);
    Serial.begin(9600);
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
      post_DHTSensorData(t_temp, h_temp, nSamples);
      nSamples = 0;
    }
 
}//end of loop()


//----Functions for Stepper motors----
void spinMotor1(int steps, float revolutions){
  int stepsPerLoop = ((sm1_rpm/60)*sm1_steps)/40; //this is to check that the motor is not at the limit every 0,05 seconds
  if(sm1_Enable==0){
    sm1_Enable =1;
    analogWrite(sm1_enableA, 255);
    analogWrite(sm1_enableB, 255);
    Serial.println("sm1 enabled");
  }
  sm1_inUse = true;
  float steps_toDo = abs(steps * revolutions);
  int step_dir = (revolutions<0)?-1:1;
  while(steps_toDo > 0 && !check_sm1Sensor((step_dir == -1)?0:1)){
    sm1.step(step_dir*stepsPerLoop);   
    steps_toDo -= stepsPerLoop;
  }
  sm1_inUse = false;
  sm1_lastTimeEnabled = millis();
}

bool check_sm1Sensor(int sideInteger){
  int value = analogRead(sm1_LimitPins[sideInteger]);
  Serial.print("TCRT500 value: ");
  Serial.println(value);
  return (value <= sm1_tcrtThreshold[sideInteger])? true:false;
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
  if (strcmp(topic, mqttCurtainsCommand)==0){
    spinMotor1(sm1_steps, mqttMessage.toFloat()); 
  }
    
}

void reconnect() {
  // Loop until we're reconnected
  while (!client1.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client1.connect(CLIENT_ID, mqttTopic_deviceStatus, 0, 1, device_offline)) {
      Serial.println("connected");
      client1.publish(mqttTopic_deviceStatus,device_online, 1);
      client1.subscribe(mqttCurtainsCommand);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client1.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}


//function for DHT temperature and humidity sensor
void post_DHTSensorData(float sample_TempTotal, float sample_HumidityTotal, int sampleSize){
  h = sample_TempTotal/sampleSize;
  t = t_temp/sampleSize;
  h_temp = 0;
  t_temp = 0;
  if(isnan(h) || isnan(t)){
    Serial.println("Failed to read DHT sensor");
  }else{
    char t_char[6];
    char h_char[6];
    char hI_char[6];
    dtostrf(h, 4,2, h_char);
    dtostrf(t, 3,2, t_char);
    dtostrf(dhtSensor.computeHeatIndex(t, h, false), 4,2, hI_char);
    client1.publish("mBedroom/sensors/tempterature/status", t_char, 1);
    client1.publish("mBedroom/sensors/humidity/status", h_char, 1);
    client1.publish("mBedroom/sensors/heatIndex/status", hI_char, 1);
    dht_lastTimeTaken = millis();
  }
}
