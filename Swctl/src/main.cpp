#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

#define relayPin D6

// Change the credentials below, so your ESP8266 connects to your router
#define WIFI_SSID "BP-HOME"
#define WIFI_PASSWORD "bp083340419"

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
// const char* mqtt_server = "192.168.1.118";
// const int mqtt_port = 1883;

// Config MQTT Server
#define MQTT_HOST "bvbhome.trueddns.com"
#define MQTT_PORT 14073
#define MQTT_USER "poo"
#define MQTT_PASSWORD "banpotbp"
#define txtMyEsp "NodeMCU WaterPump Switch Contorl."

// MQTT Topics
#define MQTT_PUB_swc "MWL/swcstate"
#define MQTT_PUB_swc_LocDT "MWL/swc/locDT"

#define MQTT_SUB_swc "MWL/swc"

unsigned int swcstate = 0;

AsyncWebServer server(80);

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//Week Days
String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Month names
String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};


void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub =mqttClient.subscribe(MQTT_SUB_swc,2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  Serial.print("  payload: ");
  
  String msgTemp;
  for (size_t i = 0; i < len; i++) {
    Serial.print(payload[i]);
    msgTemp += payload[i];
  }
  Serial.println();
  if (String(topic) == MQTT_SUB_swc) {
    Serial.print("Changing Pump to ");
    if (msgTemp == "on") {
      digitalWrite(relayPin, HIGH);
      swcstate = 1;
      Serial.print("On");
    } else if (msgTemp == "off") {
      digitalWrite(relayPin, LOW);
      swcstate = 0;
      Serial.print("Off");
    }
  }
  Serial.println();
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(9600);

  pinMode(relayPin, OUTPUT);   
  
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(MQTT_USER,MQTT_PASSWORD);
  connectToWifi();

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(25200); //Thailand +7 = 25200
    
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", txtMyEsp);
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 1 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
  
    Serial.println();
    timeClient.update();
    String formattedTime = timeClient.getFormattedTime();
    unsigned long epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime ((time_t *)&epochTime);
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon+1;
    String currentMonthName = months[currentMonth-1];
    int currentYear = ptm->tm_year+1900;
    String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
    String showDT = currentDate + "--" + formattedTime;
    Serial.printf("Current Date-Time: %s", showDT.c_str());
    Serial.println();

    uint16_t packetIdPub = mqttClient.publish(MQTT_PUB_swc_LocDT, 1, true, showDT.c_str() );
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_swc_LocDT, packetIdPub);
    Serial.printf("Message: %s \n", showDT.c_str());

    const char* txtswc;
    txtswc = swcstate?"on":"off";
    uint16_t packetIdPubswc = mqttClient.publish(MQTT_PUB_swc, 1, true, txtswc );
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_swc, packetIdPubswc);
    Serial.printf("Message: %s \n", txtswc);

  }
  
}