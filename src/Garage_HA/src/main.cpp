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
#include <LiquidCrystal_I2C.h>

#define COLUMS 20
#define ROWS   4
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

#define CLEAR_CONFIG false
#define roottopic "rainibc"
#define FWVERSION "V1.0.0"

bool shouldSaveConfig  = false;
const int CHIP_ID = ESP.getChipId();

//Output
const int REL_PUMP_OFF = D5; //GPIO14

const int SW_MAX = D6; // GPIO12
const int SW_PUMP_OFF = D7; //GPIO13

// I2C
const int I2C_CLK = D1; //GPIO05
const int I2C_DAT = D2; //GPIO04

//Ultrasonic distance
const int TRIGGER = D4; //GPIO02 (High on Boot, boot failture if pulled LOW)
const int ECHO = D0; //GPIO00 (boot failure if pulled LOW)

long duration;
int distance;

//MQTT
char mqtt_server[40] = "192.168.15.107";
char mqtt_port[6] = "1883";
char mqtt_user[40] = "user";
char mqtt_pass[40] = "password";

char mqtt_root_topic[34] = roottopic;
char mqtt_SwMax_topic[44];
char mqtt_SW_PUMP_OFF_topic[44];
char mqtt_REL_PUMP_OFF_topic[44];
char mqtt_Level_topic[44];
char mqtt_distance_topic[44];

int val_swmax = LOW;
int val_swpumpoff = LOW;
int val_rel_pump_off = LOW;
int val_level = 0;
int val_distance = 0;
int distliter = 0;

int last_val_swmax = HIGH +1; //Force sending initial status 
int last_val_swpumpoff = HIGH +1; //Force sending initial status 
int last_val_rel_pump_off = HIGH +1; //Force sending initial status 
int last_val_level = -1;  //Force sending initial status 

int last_val_distance = -1;

int counter = 0; //Update MQTT from time to time...
#define MaxSendInterval 120

WiFiClient espClient;
PubSubClient client(espClient);

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig  = true;
}

void setup() {
  pinMode(SW_PUMP_OFF, INPUT);
  pinMode(SW_MAX, INPUT);
  
  pinMode(REL_PUMP_OFF, OUTPUT);
  digitalWrite(REL_PUMP_OFF,0);
  
  pinMode(TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO, INPUT); // Sets the echoPin as an Input


  Serial.begin(115200);
  Serial.println();
  Serial.print("chip id: ");
  Serial.println(CHIP_ID);
  Serial.print("Firmware Version: ");
  Serial.println(FWVERSION);

  //LCD Setup
  if (lcd.begin(COLUMS, ROWS, LCD_5x8DOTS, D2, D1) != 1) //colums - 16, rows - 2, pixels - 5x8, SDA - D2, SCL - D1
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }

  lcd.print(F("PCF8574 is OK...")); //(F()) saves string to flash & keeps dynamic memory free
  delay(2000);

  lcd.clear();

  /* prints static text */
  lcd.setCursor(0, 0);            //set 1-st colum & 2-nd row, 1-st colum & row started at zero
  lcd.print(F("Rain IBC "));
  // lcd.print(FWVERSION);
  lcd.print(F(" ("));
  lcd.print(CHIP_ID);
  lcd.print(F(")"));
  lcd.setCursor(0, 1);            //set 1-st colum & 3-rd row, 1-st colum & row started at zero

  strcat(mqtt_root_topic, "/");
  strcat(mqtt_root_topic,  String(CHIP_ID).c_str());

  snprintf(mqtt_SwMax_topic,sizeof(mqtt_SwMax_topic),"%s/%s",mqtt_root_topic,"swmax");
  snprintf(mqtt_SW_PUMP_OFF_topic,sizeof(mqtt_SW_PUMP_OFF_topic),"%s/%s",mqtt_root_topic, "swpump");
  snprintf(mqtt_REL_PUMP_OFF_topic,sizeof(mqtt_REL_PUMP_OFF_topic),"%s/%s",mqtt_root_topic, "relpump");
  snprintf(mqtt_Level_topic,sizeof(mqtt_Level_topic),"%s/%s",mqtt_root_topic, "level");
  snprintf(mqtt_distance_topic,sizeof(mqtt_distance_topic),"%s/%s",mqtt_root_topic, "dist");


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
  lcd.print(F("connected"));

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
  client.setServer(mqtt_server, 1883);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"rovema");

  ArduinoOTA.onStart([]() {
    lcd.setCursor(0, 3);            //set 1-st colum & 2-nd row, 1-st colum & row started at zero
    lcd.print(F("Start OTA"));
  });
  ArduinoOTA.onEnd([]() {
    lcd.setCursor(0, 3);            //set 1-st colum & 2-nd row, 1-st colum & row started at zero
    lcd.print(F("End OTA"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    lcd.setCursor(0, 3);            //set 1-st colum & 2-nd row, 1-st colum & row started at zero
    lcd.printf("OTA-Progress: %u%%\r", (progress / (total / 100)));

  });
  ArduinoOTA.onError([](ota_error_t error) {
    lcd.setCursor(0, 3);            //set 1-st colum & 2-nd row, 1-st colum & row started at zero
    lcd.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) lcd.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) lcd.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) lcd.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) lcd.println("Receive Failed");
    else if (error == OTA_END_ERROR) lcd.println("End Failed");
  });

    // start the OTEthernet library with internal (flash) based storage
  ArduinoOTA.begin();
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("trying to connect to mqtt...");
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } else {
      Serial.print("connecting to mqtt broker failed, rc: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

char tmp[21];

void loop() {  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //Read Inputs
  val_swmax = digitalRead(SW_MAX);;
  val_swpumpoff = digitalRead(SW_PUMP_OFF);;
  
  float tmp_level = 0.0;
  for(int i=0; i<50;i++){
    tmp_level += (float)analogRead(A0);
  }
  val_level = (int)(tmp_level / 50.0);
  val_level = map(val_level,1020,10,0,1000);

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
  Serial.print("val_swmax: ");
  Serial.println(val_swmax);
  if((val_swmax != last_val_swmax) || (counter >= MaxSendInterval)){
    if(val_swmax == HIGH) client.publish(mqtt_SwMax_topic, "HIGH");
    else client.publish(mqtt_SwMax_topic, "LOW");
    last_val_swmax = val_swmax;
  }

  if((val_swpumpoff != last_val_swpumpoff) || (counter >= MaxSendInterval)){
    if(val_swpumpoff == HIGH) client.publish(mqtt_SW_PUMP_OFF_topic, "HIGH");
    else client.publish(mqtt_SW_PUMP_OFF_topic, "LOW");
    last_val_swpumpoff = val_swpumpoff;
  }

  //Switch Pump Relais off, if switch is turned High
  val_rel_pump_off = !val_swpumpoff;

  if((val_level != last_val_level) || (counter >= MaxSendInterval)){
    snprintf(tmp,sizeof(tmp),"%d",val_level);
    client.publish(mqtt_Level_topic, tmp);
    last_val_level = val_level;
  }
  Serial.print("Level Value: ");Serial.println(val_level);

  if((val_rel_pump_off != last_val_rel_pump_off) || (counter >= MaxSendInterval)){
    if(val_rel_pump_off == HIGH) client.publish(mqtt_REL_PUMP_OFF_topic, "HIGH");
    else client.publish(mqtt_REL_PUMP_OFF_topic, "LOW");

    digitalWrite(REL_PUMP_OFF,val_rel_pump_off);

    last_val_rel_pump_off = val_rel_pump_off;
  }
  Serial.print("Relais_Pump_Off: ");Serial.println(val_rel_pump_off);
  
  if((val_distance != last_val_distance) || (counter >= MaxSendInterval)){
    snprintf(tmp,sizeof(tmp),"%d",val_distance);
    client.publish(mqtt_distance_topic, tmp);

    last_val_distance = val_distance;
  }
  Serial.print("Distance: ");Serial.println(val_distance);
  
  //LCD routines

  lcd.setCursor(0, 1);            //set 1-st colum & 2-nd row, 1-st colum & row started at zero
  snprintf(tmp,sizeof(tmp), "Level: %4d Liter   ",val_level);
  lcd.print(tmp);

  lcd.setCursor(0, 2);            //set 1-st colum & 3-rd row, 1-st colum & row started at zero
  snprintf(tmp,sizeof(tmp), "Max=%d Auto=%d, K2=%d ",val_swmax, val_swpumpoff, val_rel_pump_off);
  lcd.print(tmp);

  lcd.setCursor(0, 3);            //set 1-st colum & 3-rd row, 1-st colum & row started at zero
  snprintf(tmp,sizeof(tmp), "Dist=%d Liter=%d      ",val_distance, distliter);
  lcd.print(tmp);
  
  counter++;
  if(counter >= (MaxSendInterval + 1)){
    counter = 0;
  }
  ArduinoOTA.handle();
  delay(500);
}