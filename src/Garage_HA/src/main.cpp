//https://medium.com/marcel-works/esp8266-zur-laufzeit-konfigurieren-3b9dfcb5dfac
#include <Arduino.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>

#include <Ethernet.h>
#define NO_OTA_PORT
#include <ArduinoOTA.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "ArduinoJson.h"
#include "WiFiManager.h"
#include "PubSubClient.h"

#include <Wire.h>

#define CLEAR_CONFIG false
#define roottopic "GarageHA"
#define FWVERSION "V1.0.0"

bool shouldSaveConfig  = false;
const int CHIP_ID = ESP.getChipId();

//Output
const int REL_OPEN_DOOR = D5; //GPIO14

const int SW_DOOR = D6; // GPIO12
const int SW_RESERVED = D7; //GPIO13

// I2C
const int I2C_CLK = D1; //GPIO05
const int I2C_DAT = D2; //GPIO04

//Ultrasonic distance
const int TRIGGER = D4; //GPIO02 (High on Boot, boot failture if pulled LOW)
const int ECHO = D0; //GPIO16 (High on Boot)

long duration;
int distance;

//MQTT
char mqtt_server[40] = "192.168.15.107";
char mqtt_port[6] = "1883";
char mqtt_user[40] = "user";
char mqtt_pass[40] = "password";

char mqtt_root_topic[34] = roottopic;
char mqtt_SwDoor_topic[44];
char mqtt_SW_Reserved_topic[44];
char mqtt_REL_Door_topic[44];
char mqtt_distance_topic[44];
char mqtt_WIFI_RSSI_topic[44];
char mqtt_SUB_REL_Door_topic[44];

int val_swdoor = LOW;
int val_swreserved = LOW;
int val_rel_open_door = LOW;
int val_distance = 0;
int distliter = 0;

int last_val_swdoor = HIGH +1; //Force sending initial status 
int last_val_swreserved = HIGH +1; //Force sending initial status 
int last_val_rel_door = HIGH +1; //Force sending initial status 
int last_val_level = -1;  //Force sending initial status 

int last_val_distance = -1;

int counter = 0; //Update MQTT from time to time...
#define MaxSendInterval 30

WiFiClient espClient;
PubSubClient client(espClient);

char tmp[21];

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig  = true;
}

void SubCallback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == mqtt_SUB_REL_Door_topic) { //"GarageHA/14087507/reldoorout"){ 
    Serial.print("Changing output to ");
    if((messageTemp == "HIGH") || (messageTemp == "1")){
      Serial.println("Rel HIGH");
      digitalWrite(REL_OPEN_DOOR, HIGH);
    }
    else if((messageTemp == "LOW") || (messageTemp == "0")){
      Serial.println("Rel LOW");
      digitalWrite(REL_OPEN_DOOR, LOW);
    }
    delay(500);
  }
}

void setup() {
  pinMode(SW_RESERVED, INPUT);
  pinMode(SW_DOOR, INPUT);
  
  pinMode(REL_OPEN_DOOR, OUTPUT);
  digitalWrite(REL_OPEN_DOOR,0);
  
  pinMode(TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO, INPUT); // Sets the echoPin as an Input


  Serial.begin(115200);
  Serial.println();
  Serial.print("chip id: ");
  Serial.println(CHIP_ID);
  Serial.print("Firmware Version: ");
  Serial.println(FWVERSION);

  strcat(mqtt_root_topic, "/");
  strcat(mqtt_root_topic,  String(CHIP_ID).c_str());

  snprintf(mqtt_SwDoor_topic,sizeof(mqtt_SwDoor_topic),"%s/%s",mqtt_root_topic,"swdoor");
  snprintf(mqtt_SW_Reserved_topic,sizeof(mqtt_SW_Reserved_topic),"%s/%s",mqtt_root_topic, "swreserved");
  snprintf(mqtt_REL_Door_topic,sizeof(mqtt_REL_Door_topic),"%s/%s",mqtt_root_topic, "reldoor");
  snprintf(mqtt_distance_topic,sizeof(mqtt_distance_topic),"%s/%s",mqtt_root_topic, "dist");
  snprintf(mqtt_WIFI_RSSI_topic,sizeof(mqtt_WIFI_RSSI_topic),"%s/%s",mqtt_root_topic, "wifirssi");
  snprintf(mqtt_SUB_REL_Door_topic,sizeof(mqtt_REL_Door_topic),"%s/%s",mqtt_root_topic, "reldoorout");
  Serial.print("Subscribing to Topic:");
  Serial.println(mqtt_SUB_REL_Door_topic);
  
  //clean FS, for testing
  if(CLEAR_CONFIG) LittleFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");
  if (LittleFS.begin()) {
    Serial.println("mounted file system");
      if (LittleFS.exists("/config.json")) {
        //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        DeserializationError error = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if (!error) {
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqtt_root_topic, json["mqtt_root_topic"]);
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount file system");
  }

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, 40);
  WiFiManagerParameter custom_mqtt_root_topic("topic", "mqtt root topic", mqtt_root_topic, 34);

  WiFiManager wifiManager;  

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_root_topic);

  //reset settings - for testing
  if(CLEAR_CONFIG) wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(120);


  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "NodeMCU-AP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("NodeMCU-AP", "12345678")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  Serial.println("connected");
  
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(mqtt_root_topic, custom_mqtt_root_topic.getValue());

  if (shouldSaveConfig ) {
    Serial.println("saving config");

    DynamicJsonDocument json(1024);
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_root_topic"] = mqtt_root_topic;

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJson(json, Serial);
    serializeJson(json, configFile);
    configFile.close();
  }

  Serial.println("assigned ip");
  Serial.println(WiFi.localIP());
  Serial.println("Signal strength");
  Serial.println(WiFi.RSSI());

  client.setServer(mqtt_server, 1883);
  client.setCallback(SubCallback);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"rovema");

  ArduinoOTA.onStart([]() {
    Serial.print(F("Start OTA"));
  });
  ArduinoOTA.onEnd([]() {
    Serial.print(F("End OTA"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA-Progress: %u%%\r", (progress / (total / 100)));

  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

    // start the OTEthernet library with internal (flash) based storage
  ArduinoOTA.begin();
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("trying to connect to mqtt...");
    if (client.connect("ESP8266Client_GHA")) {
      Serial.println("connected");

      client.subscribe(mqtt_SUB_REL_Door_topic);

    } else {
      Serial.print("connecting to mqtt broker failed, rc: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

int32_t publish_WifiRssi(void)
{
  int32_t sig = WiFi.RSSI();
  snprintf(tmp,sizeof(tmp),"%d",sig);
  client.publish(mqtt_WIFI_RSSI_topic, tmp);

  Serial.println("Signal strength");
  Serial.println(WiFi.RSSI());

  return sig;
}

void loop() {  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //Read Inputs
  val_swdoor = digitalRead(SW_DOOR);;
  val_swreserved = digitalRead(SW_RESERVED);;
  
  // Clears the trigPin
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIGGER, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO, HIGH);

  // Calculating the distance
  distance= duration*0.034/2;

  val_distance = distance;
  
  //Limits in cm
  float distmax = 20.0; //full
  float distmin = 113.0; //empty

  distliter = (val_distance - distmin) * 1000 / (distmax - distmin);
  
  if(distliter < 0) distliter = 0;
  if(distliter > 1000) distliter = 1000;

  //Send Values
  Serial.print("val_swdoor: ");
  Serial.println(val_swdoor);
  if((val_swdoor != last_val_swdoor) || (counter >= MaxSendInterval)){
    if(val_swdoor == HIGH) client.publish(mqtt_SwDoor_topic, "1");
    else client.publish(mqtt_SwDoor_topic, "0");
    last_val_swdoor = val_swdoor;

    publish_WifiRssi();
  }

  if((val_swreserved != last_val_swreserved) || (counter >= MaxSendInterval)){
    if(val_swreserved == HIGH) client.publish(mqtt_SW_Reserved_topic, "1");
    else client.publish(mqtt_SW_Reserved_topic, "0");
    last_val_swreserved = val_swreserved;

    publish_WifiRssi();
  }

  //Switch Door_Open_Rel off if it's on
  val_rel_open_door = digitalRead(REL_OPEN_DOOR);

  if((val_rel_open_door != last_val_rel_door) || (counter >= MaxSendInterval)){
    if(val_rel_open_door == HIGH) client.publish(mqtt_REL_Door_topic, "1");
    else client.publish(mqtt_REL_Door_topic, "0");

    last_val_rel_door = val_rel_open_door;

    publish_WifiRssi();

    //Switch off again, if it was HIGH
    if(val_rel_open_door == HIGH)
    {
      delay(500);
      digitalWrite(REL_OPEN_DOOR, LOW);
      client.publish(mqtt_SUB_REL_Door_topic, "0");
    }
  }
  Serial.print("Relais_Door_Open: ");Serial.println(val_rel_open_door);
  

  if((val_distance != last_val_distance) || (counter >= MaxSendInterval)){
    snprintf(tmp,sizeof(tmp),"%d",val_distance);
    client.publish(mqtt_distance_topic, tmp);

    last_val_distance = val_distance;

    publish_WifiRssi();
  }
  Serial.print("Distance: ");Serial.println(val_distance);
  
  counter++;
  if(counter >= (MaxSendInterval + 1)){
    counter = 0;
  }
  ArduinoOTA.handle();
  delay(500);
}