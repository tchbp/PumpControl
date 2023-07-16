#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>

#define trigPin D5
#define echoPin D6
//define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034
  long duration;
  float distanceCm;
  float temperature;
  bool isUs100 = true;

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
#define txtMyEsp "NodeMCU Water Level Sensor With US-100."

// MQTT Topics
#define MQTT_PUB_Dist "MWL/dist"
#define MQTT_PUB_Dist_LocDT "MWL/dist/locDT"
#define MQTT_PUB_Temp "MWL/temp"

AsyncWebServer server(80);

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

SoftwareSerial us100(echoPin,trigPin);

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings

void us100_flush() {
  while(us100.available())
  us100.read();
}

void getDistance() 
{
  if (isUs100) {
    us100_flush();
    us100.write(0x55);
    delay(100);
    if (us100.available() < 2) {
      Serial.println("Error read for US-100");
      delay(500);
      return;
    }
    distanceCm = (us100.read() * 256 + us100.read())/10;

    us100_flush();
    us100.write(0x50);
    delay(100);
    if (us100.available() < 1) {
      delay(500);
      return;
    }
    temperature = us100.read() - 45;
    Serial.print("Temperature : ");
    Serial.print(temperature);
    Serial.println(" *C.");

  } else {
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    
    // Calculate the distance
    distanceCm = duration * SOUND_VELOCITY/2;

  }
  
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
 
}

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
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
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
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(9600);
  if (isUs100) {
    us100.begin(9600);
  } else {
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  }
   
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(MQTT_USER,MQTT_PASSWORD);
  connectToWifi();
    
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

    getDistance();
    Serial.printf("Water Level Distance %.2f cm", distanceCm);
    Serial.println();

    // Publish an MQTT message
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_Dist, 1, true, String(distanceCm).c_str() );
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_Dist, packetIdPub1);
    Serial.printf("Message: %.2f  cm\n", distanceCm);

    if (isUs100) {
      // Publish an MQTT message for Temperture.
      uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_Temp, 1, true, String(temperature).c_str() );
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_Temp, packetIdPub2);
      Serial.printf("Message: %.2f *C\n", temperature);
    }

  }
  
}