#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

#ifndef BOARD_ID
#define BOARD_ID "esp8266"
#endif

#ifndef MQTT_SERVER
#define MQTT_SERVER ""
#endif

#ifndef MQTT_USERNAME
#define MQTT_USERNAME ""
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif

#ifndef MQTT_POLLING_TIMEOUT
#define MQTT_POLLING_TIMEOUT 60000
#endif

char mqtt_server[64] = MQTT_SERVER;
char mqtt_username[64] = MQTT_USERNAME;
char mqtt_password[64] = MQTT_PASSWORD;

#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"

char MQTT_TOPIC_AVAILABILITY[80];
char MQTT_TOPIC_CALLBACK[80];
char MQTT_TOPIC_DEBUG[80];

char MQTT_TOPIC_TEMPERATURE[80];
char MQTT_TOPIC_HUMIDITY[80];
char MQTT_TOPIC_MOTION[80];
char MQTT_TOPIC_SERVO[80];
char MQTT_TOPIC_UP_DOWN[80];
char MQTT_TOPIC_DESK[80];
char MQTT_TOPIC_IRDA[80];
char MQTT_TOPIC_AMBIENT_LIGHT[80];
char MQTT_TOPIC_SOIL_MOISTURE[80];

WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient;

WiFiManagerParameter wifi_param_mqtt_server("mqtt_server", "MQTT server", mqtt_server, sizeof(mqtt_server));
WiFiManagerParameter wifi_param_mqtt_username("mqtt_user", "MQTT username", mqtt_username, sizeof(mqtt_username));
WiFiManagerParameter wifi_param_mqtt_password("mqtt_pass", "MQTT password", mqtt_password, sizeof(mqtt_password));

unsigned long mqttLastTime = millis();

void sendMQTTMessage(const char *topic, const char *message, bool retained)
{
    Serial.printf("MQTT message - topic: <%s>, message: <%s> -> ", topic, message);

    if (mqttClient.publish(topic, message))
    {
        Serial.println("sent");
    }
    else
    {
        Serial.println("error");
    }
}

void logError(const char *msg)
{
    Serial.print("[ERROR] ");
    Serial.println(msg);
    sendMQTTMessage(MQTT_TOPIC_DEBUG, msg, false);
}

void saveConfig()
{
    DynamicJsonDocument json(512);
    json["mqtt_server"] = wifi_param_mqtt_server.getValue();
    json["mqtt_username"] = wifi_param_mqtt_username.getValue();
    json["mqtt_password"] = wifi_param_mqtt_password.getValue();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
        Serial.println("Failed to open config file for writing");
        return;
    }

    serializeJson(json, configFile);
    configFile.close();

    Serial.printf("Saved JSON: %s", json.as<String>().c_str());
}

void loadConfig()
{
    if (!SPIFFS.begin())
    {
        Serial.println("Failed to open SPIFFS");
        return;
    }

    if (!SPIFFS.exists("/config.json"))
    {
        Serial.println("Config file not found");
        return;
    }

    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile)
    {
        Serial.println("Failed to open config file");
        return;
    }

    const size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);

    configFile.readBytes(buf.get(), size);
    DynamicJsonDocument json(512);

    if (DeserializationError::Ok != deserializeJson(json, buf.get()))
    {
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

void setupGeneric()
{
    Serial.begin(9600);
    while (!Serial)
    {

        ; // wait for serial port to connect. Needed for native USB port only
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    loadConfig();

    Serial.printf("Board Identifier: %s\n", BOARD_ID);

    snprintf(MQTT_TOPIC_AVAILABILITY, 80, "%s/availability", BOARD_ID);
    snprintf(MQTT_TOPIC_CALLBACK, 80, "%s/callback", BOARD_ID);
    snprintf(MQTT_TOPIC_DEBUG, 80, "%s/debug", BOARD_ID);

    snprintf(MQTT_TOPIC_TEMPERATURE, 80, "%s/temperature", BOARD_ID);
    snprintf(MQTT_TOPIC_HUMIDITY, 80, "%s/humidity", BOARD_ID);
    snprintf(MQTT_TOPIC_MOTION, 80, "%s/motion", BOARD_ID);
    snprintf(MQTT_TOPIC_SERVO, 80, "%s/servo", BOARD_ID);
    snprintf(MQTT_TOPIC_IRDA, 80, "%s/irda", BOARD_ID);
    snprintf(MQTT_TOPIC_AMBIENT_LIGHT, 80, "%s/ambientlight", BOARD_ID);
    snprintf(MQTT_TOPIC_UP_DOWN, 80, "%s/updown", BOARD_ID);
    snprintf(MQTT_TOPIC_SOIL_MOISTURE, 80, "%s/soilmoisture", BOARD_ID);
    snprintf(MQTT_TOPIC_DESK, 80, "%s/desk", BOARD_ID);
}

void handleDeepSleep(DynamicJsonDocument json)
{
    uint64_t seconds = json["seconds"].as<uint64_t>();
    Serial.printf("Deep sleep for %lld seconds...\n", seconds);

    ESP.deepSleep(seconds * 1000000);
}

void handleRestart(DynamicJsonDocument json)
{
    Serial.println("Restarting...");
    ESP.restart();
}

#define WIFI_MAX_TRIES 15

void setupWifi()
{
    if (WiFi.status() == WL_NO_SHIELD)
    {

        Serial.println("WiFi shield not present");
        return;
    }

    WiFi.hostname(BOARD_ID);

#ifdef WIFI_SSID
#ifdef WIFI_PASSWORD
    Serial.printf("Connecting to WiFi (protected): %s : %s\n", WIFI_SSID, WIFI_PASSWORD);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#else
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID);
#endif

    for (int retries = 0; retries < WIFI_MAX_TRIES && WiFi.status() != WL_CONNECTED; retries++)
    {
        Serial.printf("WiFi check (%d of %d)...\n", retries + 1, WIFI_MAX_TRIES);
        delay(1000);
    }

#endif

    wifiManager.setConfigPortalBlocking(false);
    wifiManager.setDebugOutput(true);
    wifiManager.setSaveParamsCallback(saveConfig);

    wifiManager.addParameter(&wifi_param_mqtt_server);
    wifiManager.addParameter(&wifi_param_mqtt_username);
    wifiManager.addParameter(&wifi_param_mqtt_password);

    if (WiFi.status() == WL_CONNECTED || wifiManager.autoConnect(BOARD_ID))
    {
        WiFi.mode(WIFI_STA);
        wifiManager.startWebPortal();
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

void loopWifi()
{
    wifiManager.process();
}

void mqttConnect()
{
    Serial.printf("Connecting to MQTT server: %s (%s : %s)... ", mqtt_server, mqtt_username, mqtt_password);

    if (mqttClient.connect(BOARD_ID, mqtt_username, mqtt_password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE))
    {
        Serial.println("connected");
        mqttClient.subscribe(MQTT_TOPIC_CALLBACK);
        sendMQTTMessage(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
    }
    else
    {
        Serial.println("failed");
    }
}

void loopMQTT()
{
    if (mqttClient.connected())
    {
        mqttClient.loop();
        return;
    }

    if (millis() < mqttLastTime + MQTT_POLLING_TIMEOUT)
    {
        return;
    }

    Serial.println("Connection to MQTT lost, reconnecting...");
    mqttLastTime = millis();

    mqttConnect();
}

void setupOTA()
{
    ArduinoOTA.onStart([]() {});
    ArduinoOTA.onEnd([]() {});
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
    ArduinoOTA.onError([](ota_error_t error) {});
    ArduinoOTA.setHostname(BOARD_ID);
    ArduinoOTA.setPassword(BOARD_ID);
    ArduinoOTA.begin();
}

void loopOTA()
{
    ArduinoOTA.handle();
}

void setupMDNS()
{
    MDNS.begin(BOARD_ID);
}

void loopMDNS()
{
    MDNS.update();
}

#ifdef DHT_ENABLED
#include "DHTesp.h"

#ifndef DHT_POLLING_TIMEOUT
#define DHT_POLLING_TIMEOUT 60000
#endif

DHTesp dht;
unsigned long dhtLastTime = 0;

void setupDHT()
{
    dht.setup(DHT_PIN, DHTesp::DHT11);
}

void loopDHT()
{
    if (millis() < dhtLastTime + DHT_POLLING_TIMEOUT)
    {
        return;
    }

    dhtLastTime = millis();

    TempAndHumidity values = dht.getTempAndHumidity();

    Serial.printf("DHT: %f %f\n", values.temperature, values.humidity);

    if (!isnan(values.temperature))
    {
        sendMQTTMessage(MQTT_TOPIC_TEMPERATURE, String(values.temperature).c_str(), false);
    }
    else
    {
        Serial.printf("DHT temperature error: %s", dht.getStatusString());
    }

    if (!isnan(values.humidity))
    {
        sendMQTTMessage(MQTT_TOPIC_HUMIDITY, String(values.humidity).c_str(), false);
    }
    else
    {
        Serial.printf("DHT humidity error: %s", dht.getStatusString());
    }
}

#endif

#ifdef DESK_ENABLED

#ifndef DESK_POLLING_TIMEOUT
#define DESK_POLLING_TIMEOUT 1000
#endif

#ifndef DESK_STUCK_MOVING_MAX
#define DESK_STUCK_MOVING_MAX 100
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

int currentPosition = 0;
int targetPosition = 0;

bool deskActive = false;

unsigned long deskLastTime = 0;
uint8_t minDelta = 2;
int movingIndex = 0;

void setupDesk()
{
    pinMode(DESK_UP_PIN, OUTPUT);
    digitalWrite(DESK_UP_PIN, LOW);

    pinMode(DESK_DOWN_PIN, OUTPUT);
    digitalWrite(DESK_DOWN_PIN, LOW);

    pinMode(DESK_HC_ECHO_PIN, INPUT);
    pinMode(DESK_HC_TRIG_PIN, OUTPUT);
}

int getCurrentPosition()
{
    digitalWrite(DESK_HC_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(DESK_HC_TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(DESK_HC_TRIG_PIN, LOW);

    long duration = pulseIn(DESK_HC_ECHO_PIN, HIGH);
    return duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}

void handleDesk(DynamicJsonDocument json)
{
    targetPosition = json["targetPosition"].as<int>();
    deskActive = true;
}

DynamicJsonDocument state(512);

void sendDeskStatusUpdate()
{
    state["currentPosition"] = currentPosition;
    state["targetPosition"] = targetPosition;
    state["positionState"] = deskActive ? targetPosition > currentPosition ? "INCREASING" : "DECREASING" : "STOPPED";

    sendMQTTMessage(MQTT_TOPIC_DESK, state.as<String>().c_str(), false);
}

void loopDeskActive()
{
    currentPosition = getCurrentPosition();
    Serial.printf("currentPosition: %d, targetPosition: %d, Difference: %d\n\n", currentPosition, targetPosition, targetPosition - currentPosition);

    if (movingIndex > DESK_STUCK_MOVING_MAX)
    {
        Serial.printf("Desk got stuck for targetPosition = %d\n", targetPosition);
        targetPosition = currentPosition;
        // Set targetPosition to currentPosition so that we end the loop
    }

    if (abs(targetPosition - currentPosition) < minDelta)
    {
        Serial.println("Desk reached position");

        digitalWrite(DESK_UP_PIN, LOW);
        digitalWrite(DESK_DOWN_PIN, LOW);

        deskActive = false;
        movingIndex = 0;

        sendDeskStatusUpdate();
        return;
    }

    int8_t pin = targetPosition > currentPosition ? DESK_UP_PIN : DESK_DOWN_PIN;

    // Always make sure we set the opposite pin to LOW
    digitalWrite(pin == DESK_UP_PIN ? DESK_DOWN_PIN : DESK_UP_PIN, LOW);
    digitalWrite(pin, HIGH);

    movingIndex++;

    Serial.printf("Moving desk to %s, movingIndex = %d\n", pin == DESK_UP_PIN ? "UP" : "DOWN", movingIndex);
    sendDeskStatusUpdate();
}

void loopDeskIdle()
{
    if (millis() < deskLastTime + DESK_POLLING_TIMEOUT)
    {
        return;
    }

    deskLastTime = millis();

    int newCurrentPosition = getCurrentPosition();
    if (newCurrentPosition != currentPosition)
    {
        currentPosition = newCurrentPosition;
        targetPosition = newCurrentPosition;
        sendDeskStatusUpdate();
    }
}

void loopDesk()
{
    if (deskActive)
    {
        loopDeskActive();
    }
    else
    {
        loopDeskIdle();
    }
}

#endif

#ifdef PIR_ENABLED

#ifndef PIR_POLLING_TIMEOUT
#define PIR_POLLING_TIMEOUT 2000
#endif

bool pirState = false;
unsigned long pirLastTime = 0;

void setupPIR()
{
    pinMode(PIR_PIN, INPUT);
}

void loopPIR()
{
    if (millis() < pirLastTime + PIR_POLLING_TIMEOUT)
    {
        return;
    }

    pirLastTime = millis();
    int pirValue = digitalRead(PIR_PIN);

    Serial.printf("PIR: %d\n", pirValue);

    if (pirValue == HIGH)
    {
        if (pirState == false)
        {
            pirState = true;
            sendMQTTMessage(MQTT_TOPIC_MOTION, "1", false);
        }
    }
    else
    {
        if (pirState == true)
        {
            pirState = false;
            sendMQTTMessage(MQTT_TOPIC_MOTION, "0", false);
        }
    }

    digitalWrite(LED_BUILTIN, !pirState);
}
#endif

#ifdef SERVO_ENABLED
#include <Servo.h>

Servo servo;

void setupServo()
{
    servo.attach(SERVO_PIN);
    servo.write(0);
}

void loopServo()
{
}

void handleServo(DynamicJsonDocument json)
{
    String method = json["method"].as<String>();
    int degree = json["degree"].as<int>();

    if (!degree)
        degree = 30;

    if (method == "degree")
    {
        servo.write(degree);
        sendMQTTMessage(MQTT_TOPIC_SERVO, "1", false);
        return;
    }

    if (method == "push")
    {
        int delayMs = json["delayMs"].as<int>();
        int cycles = json["cycles"].as<int>();
        int delayMsBetweenCycles = json["delayMsBetweenCycles"].as<int>();

        if (!delayMs)
            delayMs = 600;
        if (!cycles)
            cycles = 1;
        if (!delayMsBetweenCycles)
            delayMsBetweenCycles = 1000;

        for (int i = 0; i < cycles; i++)
        {
            servo.write(degree);
            delay(delayMs);
            servo.write(0);

            if ((i + 1) != cycles)
            {
                delay(delayMsBetweenCycles);
            }
        }

        sendMQTTMessage(MQTT_TOPIC_SERVO, "on", false);
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

void setupSoilMoisture()
{
    pinMode(SOIL_MOISTURE_POWER_PIN, OUTPUT);
    digitalWrite(SOIL_MOISTURE_POWER_PIN, LOW);
}

void loopSoilMoisture()
{
    if (millis() < soilMoistureLastTime + SOIL_MOISTURE_POLLING_TIMEOUT)
    {
        return;
    }

    Serial.println("Reading soil moisture");

    soilMoistureLastTime = millis();

    digitalWrite(SOIL_MOISTURE_POWER_PIN, HIGH); // Turn the sensor ON
    delay(10);                                   // Allow power to settle

    int val = analogRead(A0); // Read the analog value form sensor

    digitalWrite(SOIL_MOISTURE_POWER_PIN, LOW); // Turn the sensor OFF

    Serial.printf("Soil Moisture value: %d\n", val);

    if (val == 1024)
    {
        Serial.printf("Soil Moisture sensor is disconnected");
        logError("SOIL_MOISTURE_SENSOR_DISCONNECTED");
        // return;
    }

    sendMQTTMessage(MQTT_TOPIC_SOIL_MOISTURE, String(val).c_str(), false);
}

#endif

#ifdef IRDA_ENABLED
#include <IRremote.h>

void setupIRDA()
{
#ifdef IRDA_RECV_PIN
    IrReceiver.begin(IRDA_RECV_PIN, false, false);
#endif
#ifdef IRDA_SEND_PIN
    IrSender.begin(IRDA_SEND_PIN, false, false);
#endif
}

void loopIRDA()
{
#ifdef IRDA_RECV_PIN
    if (IrReceiver.decode())
    {
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.resume();
    }
#endif
}

void handleIRDA(DynamicJsonDocument json)
{
    uint16_t address = json["address"].as<uint16_t>();
    uint8_t code = json["code"].as<uint8_t>();
    if (!address || !code)
    {
        logError("IRDA_UNDEFINED_CODE");
        return;
    }

    Serial.printf("IRDA sending: %d %d\n", address, code);

    digitalWrite(LED_BUILTIN, LOW);

    IrSender.sendNEC(address, code, 1);
    sendMQTTMessage(MQTT_TOPIC_IRDA, "on", false);

    digitalWrite(LED_BUILTIN, HIGH);
}
#endif

#ifdef AMBIENT_LIGHT_ENABLED

#ifndef AMBIENT_LIGHT_POLLING_TIMEOUT
#define AMBIENT_LIGHT_POLLING_TIMEOUT 2000
#endif

unsigned long ambientLightLastTime = 0;

void setupAmbientLight()
{
}

void loopAmbientLight()
{
    if (millis() < ambientLightLastTime + AMBIENT_LIGHT_POLLING_TIMEOUT)
    {
        return;
    }

    ambientLightLastTime = millis();

    int value = analogRead(A0);
    Serial.printf("Ambient light: %d\n", value);

    sendMQTTMessage(MQTT_TOPIC_AMBIENT_LIGHT, String(value).c_str(), false);
}

#endif

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char payloadText[length + 1];
    snprintf(payloadText, length + 1, "%s", payload);

    Serial.printf("MQTT callback with topic <%s> and payload <%s>\n", topic, payloadText);
    // sendMQTTMessage(MQTT_TOPIC_DEBUG, payloadText, false);

    DynamicJsonDocument commandJson(256);
    DeserializationError err = deserializeJson(commandJson, payloadText);

    if (err)
    {
        logError("INVALID_JSON");
        return;
    }

    String command = commandJson["command"].as<String>();

    if (command == "availability")
    {
        sendMQTTMessage(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
        return;
    }

    if (command == "deepsleep")
    {
        handleDeepSleep(commandJson);
        return;
    }

    if (command == "restart")
    {
        handleRestart(commandJson);
        return;
    }

#ifdef UP_DOWN_ENABLED
    if (command == "updown")
    {
        handleUpDown(commandJson);
        return;
    }
#endif

#ifdef SERVO_ENABLED
    if (command == "servo")
    {
        handleServo(commandJson);
        return;
    }
#endif

#ifdef IRDA_ENABLED
    if (command == "irda")
    {
        handleIRDA(commandJson);
        return;
    }
#endif

#ifdef DESK_ENABLED
    if (command == "desk")
    {
        handleDesk(commandJson);
        return;
    }
#endif

    Serial.printf("Unknown callback command: %s", command.c_str());
    logError("MQTT_INVALID_COMMAND");
}

// -------------------

void setupMQTT()
{
    mqttClient.setClient(wifiClient);

    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    mqttConnect();
}

void setup()
{
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
    setupServo();
#endif
#ifdef IRDA_ENABLED
    setupIRDA();
#endif
#ifdef AMBIENT_LIGHT_ENABLED
    setupAmbientLight();
#endif
#ifdef UP_DOWN_ENABLED
    setupUpDown();
#endif
#ifdef SOIL_MOISTURE_ENABLED
    setupSoilMoisture();
#endif
#ifdef DESK_ENABLED
    setupDesk();
#endif
}

void loop()
{
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
#ifdef UP_DOWN_ENABLED
    loopUpDown();
#endif
#ifdef SOIL_MOISTURE_ENABLED
    loopSoilMoisture();
#endif
#ifdef DESK_ENABLED
    loopDesk();
#endif
}