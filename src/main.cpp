#include "Arduino.h"
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

#ifndef BOARD_ID
#define BOARD_ID "esp8266"
#endif

#ifndef BOARD_NAME
#define BOARD_NAME "ESP8266"
#endif

#ifndef MQTT_POLLING_TIMEOUT
#define MQTT_POLLING_TIMEOUT 60000
#endif

#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"

char mqtt_server[64] = "";
char mqtt_username[64] = "";
char mqtt_password[64] = "";

String MQTT_TOPIC_AVAILABILITY = String(BOARD_ID) + String("/availability");
String MQTT_TOPIC_CALLBACK = String(BOARD_ID) + String("/callback");
String MQTT_TOPIC_DEBUG = String(BOARD_ID) + String("/debug");

WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient;

WiFiManagerParameter wifi_param_mqtt_server("mqtt_server", "MQTT server",
                                            mqtt_server, sizeof(mqtt_server));
WiFiManagerParameter wifi_param_mqtt_username("mqtt_user", "MQTT username",
                                              mqtt_username,
                                              sizeof(mqtt_username));
WiFiManagerParameter wifi_param_mqtt_password("mqtt_pass", "MQTT password",
                                              mqtt_password,
                                              sizeof(mqtt_password));

unsigned long mqttLastTime = millis();

void sendMQTTMessage(const char *topic, const char *message, bool retained) {
  Serial.printf("MQTT message - topic: <%s>, message: <%s> -> ", topic,
                message);

  if (mqttClient.publish(topic, message)) {
    Serial.println("sent");
  } else {
    Serial.println("error");
  }
}

void logError(const char *msg) {
  Serial.print("[ERROR] ");
  Serial.println(msg);
  sendMQTTMessage(MQTT_TOPIC_DEBUG.c_str(), msg, false);
}

void saveConfig() {
  DynamicJsonDocument json(512);
  json["mqtt_server"] = wifi_param_mqtt_server.getValue();
  json["mqtt_username"] = wifi_param_mqtt_username.getValue();
  json["mqtt_password"] = wifi_param_mqtt_password.getValue();

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }

  serializeJson(json, configFile);
  configFile.close();

  Serial.printf("Saved JSON: %s", json.as<String>().c_str());
}

void loadConfig() {
  if (!SPIFFS.begin()) {
    Serial.println("Failed to open SPIFFS");
    return;
  }

  if (!SPIFFS.exists("/config.json")) {
    Serial.println("Config file not found");
    return;
  }

  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return;
  }

  const size_t size = configFile.size();
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);
  DynamicJsonDocument json(512);

  if (DeserializationError::Ok != deserializeJson(json, buf.get())) {
    Serial.println("Failed to parse config fileDebug");
    return;
  }

  strcpy(mqtt_server, json["mqtt_server"]);
  strcpy(mqtt_username, json["mqtt_username"]);
  strcpy(mqtt_password, json["mqtt_password"]);

  wifi_param_mqtt_server.setValue(mqtt_server, sizeof(mqtt_server));
  wifi_param_mqtt_username.setValue(mqtt_username, sizeof(mqtt_username));
  wifi_param_mqtt_password.setValue(mqtt_password, sizeof(mqtt_password));
}

void setupGeneric() {
  Serial.begin(9600);
  while (!Serial) {

    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  loadConfig();

  Serial.printf("Board Identifier: %s\n", BOARD_ID);
}

void handleDeepSleep(DynamicJsonDocument json) {
  uint64_t seconds = json["seconds"].as<uint64_t>();
  Serial.printf("Deep sleep for %lld seconds...\n", seconds);

  ESP.deepSleep(seconds * 1000000);
}

void handleRestart(DynamicJsonDocument json) {
  Serial.println("Restarting...");
  ESP.restart();
}

void setupWifi() {
  if (WiFi.status() == WL_NO_SHIELD) {

    Serial.println("WiFi shield not present");
    return;
  }

  WiFi.hostname(BOARD_ID);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setDebugOutput(true);
  wifiManager.setSaveParamsCallback(saveConfig);

  wifiManager.addParameter(&wifi_param_mqtt_server);
  wifiManager.addParameter(&wifi_param_mqtt_username);
  wifiManager.addParameter(&wifi_param_mqtt_password);

  if (WiFi.status() == WL_CONNECTED || wifiManager.autoConnect(BOARD_ID)) {
    WiFi.mode(WIFI_STA);
    wifiManager.startWebPortal();
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void loopWifi() { wifiManager.process(); }

#ifdef HA_ENABLED
void sendHAAutoDiscovery() {
  DynamicJsonDocument doc(512);
  doc["name"] = BOARD_NAME;
  doc["unique_id"] = BOARD_ID;
  doc["command_topic"] = MQTT_TOPIC_CALLBACK;
  doc["command_template"] = "{\"command\":\"servo\", \"degree\":140, \"method\":\"push\"}";
  doc["availability_topic"] = MQTT_TOPIC_AVAILABILITY;

  JsonObject device = doc.createNestedObject("device");
  device["configuration_url"] = "http://" + WiFi.localIP().toString();
  device["identifiers"] = BOARD_ID;
  device["manufacturer"] = HA_MANUFACTURER;
  device["model"] = HA_MODEL;
  device["name"] = BOARD_NAME;
  device["connections"] = "[\"mac\", \"" + WiFi.macAddress() + "\"]";

  String topic = String("homeassistant/button/") + String(BOARD_ID) + String("/config");
  sendMQTTMessage(topic.c_str(), doc.as<String>().c_str(), true);
}
#endif

void mqttConnect() {
  Serial.printf("Connecting to MQTT server: %s (%s : %s)... ", mqtt_server,
                mqtt_username, mqtt_password);

  if (mqttClient.connect(BOARD_ID, mqtt_username, mqtt_password,
                         MQTT_TOPIC_AVAILABILITY.c_str(), 1, true,
                         AVAILABILITY_OFFLINE)) {
    Serial.println("connected");

    // Subscribe to the callback topic
    mqttClient.subscribe(MQTT_TOPIC_CALLBACK.c_str());

#ifdef HA_ENABLED
    // Subscribe to HA restart and send the availability message
    mqttClient.subscribe("homeassistant/status");
    sendHAAutoDiscovery();
#endif

    sendMQTTMessage(MQTT_TOPIC_AVAILABILITY.c_str(), AVAILABILITY_ONLINE, true);
  } else {
    Serial.println("failed");
  }
}

void loopMQTT() {
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }

  if (millis() < mqttLastTime + MQTT_POLLING_TIMEOUT) {
    return;
  }

  Serial.println("Connection to MQTT lost, reconnecting...");
  mqttLastTime = millis();

  mqttConnect();
}

void setupOTA() {
  ArduinoOTA.onStart([]() {});
  ArduinoOTA.onEnd([]() {});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
  ArduinoOTA.onError([](ota_error_t error) {});
  ArduinoOTA.setHostname(BOARD_ID);
  ArduinoOTA.setPassword(BOARD_ID);
  ArduinoOTA.begin();
}

void loopOTA() { ArduinoOTA.handle(); }

void setupMDNS() { MDNS.begin(BOARD_ID); }

void loopMDNS() { MDNS.update(); }

#ifdef DHT_ENABLED
#include "DHTesp.h"

#ifndef DHT_POLLING_TIMEOUT
#define DHT_POLLING_TIMEOUT 60000
#endif

DHTesp dht;
unsigned long dhtLastTime = 0;

void setupDHT() { dht.setup(DHT_PIN, DHTesp::DHT11); }

void loopDHT() {
  if (millis() < dhtLastTime + DHT_POLLING_TIMEOUT) {
    return;
  }

  dhtLastTime = millis();

  TempAndHumidity values = dht.getTempAndHumidity();

  Serial.printf("DHT: %f %f\n", values.temperature, values.humidity);

  if (!isnan(values.temperature)) {
    sendMQTTMessage(String(String(BOARD_ID) + String("/temperature")).c_str(), String(values.temperature).c_str(), false);
  } else {
    Serial.printf("DHT temperature error: %s", dht.getStatusString());
  }

  if (!isnan(values.humidity)) {
    sendMQTTMessage(String(String(BOARD_ID) + String("/humidity")).c_str(), String(values.humidity).c_str(),
                    false);
  } else {
    Serial.printf("DHT humidity error: %s", dht.getStatusString());
  }
}

#endif

#ifdef DESK_ENABLED

#ifndef DESK_POLLING_TIMEOUT
#define DESK_POLLING_TIMEOUT 4000
#endif

#ifndef DESK_STUCK_MOVING_MAX
#define DESK_STUCK_MOVING_MAX 100
#endif

#ifndef DESK_ACTIVE_DIFF_TOLERANCE
#define DESK_ACTIVE_DIFF_TOLERANCE 1
#endif

#ifndef DESK_IDLE_DIFF_TOLERANCE
#define DESK_IDLE_DIFF_TOLERANCE 2
#endif

#ifndef DESK_UP_PIN
#error Please define DESK_UP_PIN
#endif

#ifndef DESK_DOWN_PIN
#error Please define DESK_DOWN_PIN
#endif

#ifndef DESK_HC_ECHO_PIN
#error Please define DESK_HC_ECHO_PIN
#endif

#ifndef DESK_HC_TRIG_PIN
#error Please define DESK_HC_TRIG_PIN
#endif

float currentPosition = 0;
float targetPosition = 0;
bool deskActive = false;

bool deskManualPress = false;
unsigned long manualPressFinalMillis = 0;
short manualPressPin = 0;

unsigned long deskLastTime = 0;
int movingIndex = 0;

/**
 * Configure input pins for the distance sensor and output pins to move the desk
 */
void setupDesk() {
  pinMode(DESK_UP_PIN, OUTPUT);
  digitalWrite(DESK_UP_PIN, LOW);

  pinMode(DESK_DOWN_PIN, OUTPUT);
  digitalWrite(DESK_DOWN_PIN, LOW);

  pinMode(DESK_HC_ECHO_PIN, INPUT);
  pinMode(DESK_HC_TRIG_PIN, OUTPUT);
}

/**
 * Returns the current height of the desk stated by the distance sensor
 */
float getCurrentPosition() {
  digitalWrite(DESK_HC_TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(DESK_HC_TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(DESK_HC_TRIG_PIN, LOW);

  long duration = pulseIn(DESK_HC_ECHO_PIN, HIGH);
  return duration * 0.034f /
         2.0f; // Speed of sound wave divided by 2 (go and back)
}

void handleDesk(DynamicJsonDocument json) {
  if (json["targetPosition"]) {
    targetPosition = json["targetPosition"].as<float>();
    deskActive = true;
  } else if (json["manualPress"]) {
    long manualPressCmd = json["manualPress"].as<long>();
    manualPressFinalMillis = millis() + abs(manualPressCmd);
    manualPressPin = manualPressCmd > 0 ? DESK_UP_PIN : DESK_DOWN_PIN;
    deskManualPress = true;
  }
}

DynamicJsonDocument state(512);

void sendDeskStatusUpdate() {
  state["currentPosition"] = int(currentPosition);
  state["targetPosition"] = int(targetPosition);
  state["positionState"] =
      deskActive
          ? targetPosition > currentPosition ? "INCREASING" : "DECREASING"
          : "STOPPED";
  state["manualPress"] = deskManualPress ? "on" : "off";
  state["movingIndex"] = movingIndex;

  sendMQTTMessage(String(String(BOARD_ID) + "/desk").c_str(), state.as<String>().c_str(), false);
}

void loopDeskManualPress() {
  unsigned long now = millis();

  if (deskManualPress == false || manualPressFinalMillis == 0 ||
      now > manualPressFinalMillis) {
    deskManualPress = false;

    // finish
    digitalWrite(DESK_UP_PIN, LOW);
    digitalWrite(DESK_DOWN_PIN, LOW);
  } else {
    // Always make sure we set the opposite pin to LOW
    digitalWrite(manualPressPin == DESK_UP_PIN ? DESK_DOWN_PIN : DESK_UP_PIN,
                 LOW);
    digitalWrite(manualPressPin, HIGH);
  }
  sendDeskStatusUpdate();
}

void loopDeskActive() {
  currentPosition = getCurrentPosition();
  Serial.printf("currentPosition: %f, targetPosition: %f, Difference: %f, "
                "movingIndex: %d\n\n",
                currentPosition, targetPosition,
                targetPosition - currentPosition, movingIndex);

  if (movingIndex > DESK_STUCK_MOVING_MAX) {
    Serial.printf("Desk got stuck for targetPosition = %f\n", targetPosition);
    targetPosition = currentPosition;
    // Set targetPosition to currentPosition so that we end the loop
  }

  if (abs(targetPosition - currentPosition) < DESK_ACTIVE_DIFF_TOLERANCE) {
    digitalWrite(DESK_UP_PIN, LOW);
    digitalWrite(DESK_DOWN_PIN, LOW);

    deskActive = false;
    movingIndex = 0;

    Serial.println("Desk reached position");
  } else {
    int8_t pin = targetPosition > currentPosition ? DESK_UP_PIN : DESK_DOWN_PIN;

    // Always make sure we set the opposite pin to LOW
    digitalWrite(pin == DESK_UP_PIN ? DESK_DOWN_PIN : DESK_UP_PIN, LOW);
    digitalWrite(pin, HIGH);

    movingIndex++;

    Serial.printf("Moving desk to %s, movingIndex = %d\n",
                  pin == DESK_UP_PIN ? "UP" : "DOWN", movingIndex);
  }

  sendDeskStatusUpdate();
}

void loopDeskIdle() {
  if (millis() < deskLastTime + DESK_POLLING_TIMEOUT) {
    return;
  }

  deskLastTime = millis();

  float oldCurrentPosition = currentPosition;

  currentPosition = getCurrentPosition();
  targetPosition = currentPosition;

  if (abs(currentPosition - oldCurrentPosition) > DESK_IDLE_DIFF_TOLERANCE) {
    sendDeskStatusUpdate();
  }
}

void loopDesk() {
  if (deskActive) {
    loopDeskActive();
  } else if (deskManualPress) {
    loopDeskManualPress();
  } else {
    loopDeskIdle();
  }
}

#endif

#ifdef DESK_SIMPLE_ENABLED

#ifndef DESK_UP_PIN
#error Please define DESK_UP_PIN
#endif

#ifndef DESK_DOWN_PIN
#error Please define DESK_DOWN_PIN
#endif

bool deskSimpleManualPress = false;
unsigned long deskSimplemanualPressFinalMillis = 0;
unsigned short deskSimpleManualPressPin = -1;

/**
 * Configure input pins for the distance sensor and output pins to move the desk
 */
void setupDeskSimple() {
  // Set the pins as outputs
  pinMode(DESK_UP_PIN, OUTPUT);
  pinMode(DESK_DOWN_PIN, OUTPUT);

  // Set the pins to HIGH initially (not shorted to ground)
  digitalWrite(DESK_UP_PIN, HIGH);
  digitalWrite(DESK_DOWN_PIN, HIGH);
}

void handleDeskSimple(DynamicJsonDocument json) {
  if (json["manualPress"]) {
    long manualPressCmd = json["manualPress"].as<long>();
    deskSimplemanualPressFinalMillis = millis() + abs(manualPressCmd);
    deskSimpleManualPressPin = manualPressCmd > 0 ? DESK_UP_PIN : DESK_DOWN_PIN;
    deskSimpleManualPress = true;
  }
}

void loopDeskSimple() {
  unsigned long now = millis();

  if (deskSimpleManualPress == false || deskSimpleManualPressFinalMillis == 0 ||
      now > deskSimpleManualPressFinalMillis) {
    deskSimpleManualPress = false;

    // finish
    digitalWrite(DESK_UP_PIN, HIGH);
    digitalWrite(DESK_DOWN_PIN, HIGH);
  } else {
    // Always make sure we set the opposite pin to LOW
    digitalWrite(deskSimpleManualPressPin == DESK_UP_PIN ? DESK_DOWN_PIN
                                                         : DESK_UP_PIN,
                 HIGH);
    digitalWrite(deskSimpleManualPressPin, LOW);
  }
}

#endif

#ifdef PIR_ENABLED

#ifndef PIR_POLLING_TIMEOUT
#define PIR_POLLING_TIMEOUT 2000
#endif

bool pirState = false;
unsigned long pirLastTime = 0;

void setupPIR() { pinMode(PIR_PIN, INPUT); }

void loopPIR() {
  if (millis() < pirLastTime + PIR_POLLING_TIMEOUT) {
    return;
  }

  pirLastTime = millis();
  int pirValue = digitalRead(PIR_PIN);

  Serial.printf("PIR: %d\n", pirValue);

  if (pirValue == HIGH) {
    if (pirState == false) {
      pirState = true;
      sendMQTTMessage(String(String(BOARD_ID) + String("/motion")).c_str(), "1", false);
    }
  } else {
    if (pirState == true) {
      pirState = false;
      sendMQTTMessage(String(String(BOARD_ID) + String("/motion")).c_str(), "0", false);
    }
  }

  digitalWrite(LED_BUILTIN, !pirState);
}
#endif

#ifdef SERVO_ENABLED
#include <Servo.h>

#define SERVO_PIN 5

Servo servo;

void setupServos() { servo.attach(SERVO_PIN); }

void loopServo() {}

void handleServo(DynamicJsonDocument json) {
  String method = json["method"].as<String>();
  int degree = json["degree"].as<int>();

  if (!degree)
    degree = 30;

  if (method == "ms") {
    servo.writeMicroseconds(degree);
    sendMQTTMessage(String(String(BOARD_ID) + String("/servo")).c_str(), "1", false);
    return;
  }

  if (method == "degree") {
    servo.write(degree);
    sendMQTTMessage(String(String(BOARD_ID) + String("/servo")).c_str(), "1", false);
    return;
  }

  if (method == "push") {
    int delayMs = json["delayMs"].as<int>();
    int cycles = json["cycles"].as<int>();
    int delayMsBetweenCycles = json["delayMsBetweenCycles"].as<int>();

    if (!delayMs)
      delayMs = 600;
    if (!cycles)
      cycles = 1;
    if (!delayMsBetweenCycles)
      delayMsBetweenCycles = 1000;

    for (int i = 0; i < cycles; i++) {
      servo.write(degree);
      delay(delayMs);
      servo.write(0);

      if ((i + 1) != cycles) {
        delay(delayMsBetweenCycles);
      }
    }

    sendMQTTMessage(String(String(BOARD_ID) + String("/servo")).c_str(), "on", false);
    return;
  }

  Serial.printf("Unknown servo method: %s", method.c_str());
}
#endif

#ifdef SOIL_MOISTURE_ENABLED

#ifndef SOIL_MOISTURE_POLLING_TIMEOUT
#define SOIL_MOISTURE_POLLING_TIMEOUT 600000
#endif

#ifndef SOIL_MOISTURE_POWER_PIN
#define SOIL_MOISTURE_POWER_PIN 5
#endif

unsigned long soilMoistureLastTime = -SOIL_MOISTURE_POLLING_TIMEOUT;

void setupSoilMoisture() {
  pinMode(SOIL_MOISTURE_POWER_PIN, OUTPUT);
  digitalWrite(SOIL_MOISTURE_POWER_PIN, LOW);
}

void loopSoilMoisture() {
  if (millis() < soilMoistureLastTime + SOIL_MOISTURE_POLLING_TIMEOUT) {
    return;
  }

  Serial.println("Reading soil moisture");

  soilMoistureLastTime = millis();

  digitalWrite(SOIL_MOISTURE_POWER_PIN, HIGH); // Turn the sensor ON
  delay(10);                                   // Allow power to settle

  int val = analogRead(A0); // Read the analog value form sensor

  digitalWrite(SOIL_MOISTURE_POWER_PIN, LOW); // Turn the sensor OFF

  Serial.printf("Soil Moisture value: %d\n", val);

  if (val == 1024) {
    Serial.printf("Soil Moisture sensor is disconnected");
    logError("SOIL_MOISTURE_SENSOR_DISCONNECTED");
    return;
  }

  sendMQTTMessage(String(String(BOARD_ID) + String("/moisture")).c_str(), String(val).c_str(), false);
}

#endif

#ifdef IRDA_ENABLED
#include <IRremote.h>

void setupIRDA() {
#ifdef IRDA_RECV_PIN
  IrReceiver.begin(IRDA_RECV_PIN, false, false);
#endif
#ifdef IRDA_SEND_PIN
  IrSender.begin(IRDA_SEND_PIN, false, false);
#endif
}

void loopIRDA() {
#ifdef IRDA_RECV_PIN
  if (IrReceiver.decode()) {
    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.resume();
  }
#endif
}

void handleIRDA(DynamicJsonDocument json) {
  uint16_t address = json["address"].as<uint16_t>();
  uint8_t code = json["code"].as<uint8_t>();
  if (!address || !code) {
    logError("IRDA_UNDEFINED_CODE");
    return;
  }

  Serial.printf("IRDA sending: %d %d\n", address, code);

  digitalWrite(LED_BUILTIN, LOW);

  IrSender.sendNEC(address, code, 1);
  sendMQTTMessage(String(String(BOARD_ID) + String("/irda")).c_str(), "on", false);

  digitalWrite(LED_BUILTIN, HIGH);
}
#endif

#ifdef AMBIENT_LIGHT_ENABLED

#ifndef AMBIENT_LIGHT_POLLING_TIMEOUT
#define AMBIENT_LIGHT_POLLING_TIMEOUT 2000
#endif

unsigned long ambientLightLastTime = 0;

void setupAmbientLight() {}

void loopAmbientLight() {
  if (millis() < ambientLightLastTime + AMBIENT_LIGHT_POLLING_TIMEOUT) {
    return;
  }

  ambientLightLastTime = millis();

  int value = analogRead(A0);
  Serial.printf("Ambient light: %d\n", value);

  sendMQTTMessage(String(String(BOARD_ID) + String("/ambient_light")).c_str(), String(value).c_str(), false);
}

#endif

#define LIGHT_ENABLED 1

#ifdef LIGHT_ENABLED

#ifndef LIGHT_PIN
#define LIGHT_PIN 5
#endif

short brightness = 0;

void setBrightness(int value) {
  brightness = value;
  analogWrite(LIGHT_PIN, map(value, 0, 100, 0, 255));

  sendMQTTMessage(String(String(BOARD_ID) + String("/light")).c_str(), String(value).c_str(), false);

  // Save the brightness to SPIFFS
  File lightValueFile = SPIFFS.open("/light.txt", "w");
  if (lightValueFile) {
    lightValueFile.print(brightness);
    lightValueFile.close();
  } else {
    Serial.println("Failed to open light file for writing");
  }
}

void setupLight() {
  pinMode(LIGHT_PIN, OUTPUT);

  int previousBrightness = 0;

  File lightValueFile = SPIFFS.open("/light.txt", "r");
  if (lightValueFile) {
    previousBrightness = lightValueFile.parseInt();
    lightValueFile.close();
  }

  setBrightness(previousBrightness);
}

void loopLight() {}

void handleLight(DynamicJsonDocument json) {
  String method = json["method"].as<String>();
  int previousBrightness = map(brightness, 0, 100, 0, 255);

  if (method == "on") {
    if (brightness == 0)
      brightness = 100;
    setBrightness(brightness);
    return;
  }

  if (method == "off") {
    setBrightness(0);
    return;
  }

  if (method == "brightness") {
    int value = json["brightness"].as<int>();
    setBrightness(value);
    return;
  }

  if (method == "pulse") {
    int pulses = json["pulses"].as<int>();
    if (!pulses)
      pulses = 3;

    int delayMs = json["delayMs"].as<int>();
    if (!delayMs)
      delayMs = 2;

    int b = previousBrightness;

    for (int i = 0; i < pulses; i++) {
      for (; b <= 255; b++) {
        analogWrite(LIGHT_PIN, b);
        delay(delayMs);
      }
      for (; b >= 5; b--) {
        analogWrite(LIGHT_PIN, b);
        delay(delayMs);
      }
    }
    for (; b <= previousBrightness; b++) {
      analogWrite(LIGHT_PIN, b);
      delay(delayMs);
    }

    return;
  }

  if (method == "tick") {
    int ticks = json["ticks"].as<int>();
    if (!ticks)
      ticks = 3;

    for (int i = 0; i < ticks; i++) {
      analogWrite(LIGHT_PIN, 255);
      delay(500);
      analogWrite(LIGHT_PIN, 0);
      delay(500);
    }

    delay(1000);
    analogWrite(LIGHT_PIN, previousBrightness);

    return;
  }

  Serial.printf("Unknown light method: %s", method.c_str());
}

#endif

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char payloadText[length + 1];
  snprintf(payloadText, length + 1, "%s", payload);

  Serial.printf("MQTT callback with topic <%s> and payload <%s>\n", topic,
                payloadText);

#ifdef HA_ENABLED
  if (strcmp(topic, "homeassistant/status") == 0) {
    if (strcmp(payloadText, "online") == 0) {
      sendHAAutoDiscovery();
      sendMQTTMessage(MQTT_TOPIC_AVAILABILITY.c_str(), AVAILABILITY_ONLINE, true);
      return;
    }
  }
#endif

  // Check if it's our topic
  if (strcmp(topic, MQTT_TOPIC_CALLBACK.c_str()) == 0) {
    DynamicJsonDocument commandJson(256);
    DeserializationError err = deserializeJson(commandJson, payloadText);

    if (err) {
      logError("INVALID_JSON");
      return;
    }

    String command = commandJson["command"].as<String>();

    if (command == "availability") {
      sendMQTTMessage(MQTT_TOPIC_AVAILABILITY.c_str(), AVAILABILITY_ONLINE, true);
      return;
    }

    if (command == "deepsleep") {
      handleDeepSleep(commandJson);
      return;
    }

    if (command == "restart") {
      handleRestart(commandJson);
      return;
    }

  #ifdef SERVO_ENABLED
    if (command == "servo") {
      handleServo(commandJson);
      return;
    }
  #endif

  #ifdef IRDA_ENABLED
    if (command == "irda") {
      handleIRDA(commandJson);
      return;
    }
  #endif

  #ifdef DESK_ENABLED
    if (command == "desk") {
      handleDesk(commandJson);
      return;
    }
  #endif

  #ifdef LIGHT_ENABLED
    if (command == "light") {
      handleLight(commandJson);
      return;
    }
  #endif

    Serial.printf("Unknown callback command: %s", command.c_str());
    logError("MQTT_INVALID_COMMAND");
  }
}

// -------------------

void setupMQTT() {
  mqttClient.setClient(wifiClient);

  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setKeepAlive(10);
  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(mqttCallback);

  mqttConnect();
}

void setup() {
  setupGeneric();

  setupWifi();
  setupMDNS();
  setupOTA();
  setupMQTT();

#ifdef PIR_ENABLED
  setupPIR();
#endif
#ifdef DHT_ENABLED
  setupDHT();
#endif
#ifdef SERVO_ENABLED
  setupServos();
#endif
#ifdef IRDA_ENABLED
  setupIRDA();
#endif
#ifdef AMBIENT_LIGHT_ENABLED
  setupAmbientLight();
#endif
#ifdef SOIL_MOISTURE_ENABLED
  setupSoilMoisture();
#endif
#ifdef DESK_ENABLED
  setupDesk();
#endif
#ifdef DESK_SIMPLE_ENABLED
  setupDeskSimple();
#endif
#ifdef LIGHT_ENABLED
  setupLight();
#endif
}

void loop() {
  loopWifi();
  loopMDNS();
  loopOTA();
  loopMQTT();

#ifdef PIR_ENABLED
  loopPIR();
#endif
#ifdef DHT_ENABLED
  loopDHT();
#endif
#ifdef SERVO_ENABLED
  loopServo();
#endif
#ifdef IRDA_ENABLED
  loopIRDA();
#endif
#ifdef AMBIENT_LIGHT_ENABLED
  loopAmbientLight();
#endif
#ifdef SOIL_MOISTURE_ENABLED
  loopSoilMoisture();
#endif
#ifdef DESK_ENABLED
  loopDesk();
#endif
#ifdef DESK_SIMPLE_ENABLED
  loopDeskSimple();
#endif
#ifdef LIGHT_ENABLED
  loopLight();
#endif
}